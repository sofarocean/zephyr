/** @file
 * @brief Bristlemouth Serial driver
 *
 * A Bristlemouth Serial driver to send and receive Manchester Encoded packets
 * using UART DMA
 */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME bm_stm32_hal
#define LOG_LEVEL CONFIG_NET_L2_BRISTLEMOUTH_DRIVER_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/crc.h>
#include <zephyr/random/rand32.h>

#include "ipv6.h"
#include "net_private.h"

#include "bm_stm32_hal_priv.h"
#include "murmur_hash.h"

// Per-Serial Device Context
static bm_ctx_t dev_ctx[CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT];

// ========================
// Utility methods

static inline void gen_random_mac(uint8_t *mac_addr, uint8_t b0, uint8_t b1, uint8_t b2)
{
	uint32_t entropy;

	entropy = sys_rand32_get();

	mac_addr[0] = b0;
	mac_addr[1] = b1;
	mac_addr[2] = b2;

	/* Set MAC address locally administered, unicast (LAA) */
	mac_addr[0] |= 0x02;

	mac_addr[3] = (entropy >> 16) & 0xff;
	mac_addr[4] = (entropy >>  8) & 0xff;
	mac_addr[5] = (entropy >>  0) & 0xff;
}

#if defined(CONFIG_BM_STM32_HAL_RANDOM_MAC)
static void generate_mac(uint8_t *mac_addr)
{
	gen_random_mac(mac_addr, SOFAR_OUI_B0, SOFAR_OUI_B1, SOFAR_OUI_B2);
}
#endif

// ==============================================================================

static void tx_dma_timer_handler(struct k_timer *tmr)
{
    int i;

    for (i = 0; i < CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT; ++i)
    {
        if ( tmr == &dev_ctx[i].timer)
        {
            k_sem_give((struct k_sem*) &dev_ctx[i].sem);
            break;
        }
    }
}

static void bm_serial_dma_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    int i;
    uint8_t dev_idx;
    size_t buf_len;
    bm_ctx_t* device_context = NULL;

    unsigned int irq_lock_key = irq_lock();

    struct bm_stm32_hal_dev_data* dev_data = (struct bm_stm32_hal_dev_data*)user_data;

    // Find the associated device index
    for (i = 0; i < CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT; ++i)
    {
        if ( dev == dev_ctx[i].serial_dev)
        {
            device_context  = &dev_ctx[i];
            dev_idx         = (uint8_t)i;
            buf_len         = sizeof( device_context->buf );
            break;
        }
    }
    if( device_context == NULL )
    {
        LOG_ERR( "No valid device context in DMA CB" );
        return;
    }

    // Handle event based on type
    switch (evt->type)
    {
        case UART_TX_DONE:
        case UART_TX_ABORTED:
        {
            // Start an interframe delay timer for the associated bm_uart interface
            k_timer_start( (struct k_timer*)&device_context->timer, K_USEC(device_context->interframe_delay_us), K_NO_WAIT);
            break;
        }

        case UART_RX_RDY:
        {
            size_t len              = evt->data.rx.len;
            size_t tail             = evt->data.rx.offset;
            uint8_t* buf            = evt->data.rx.buf;

            if( len > BM_MAX_WIRE_FRAME_SIZE )
            {
                // Drop packet
                LOG_ERR( "Received frame was too large (%" PRIu32 "B) - dropping", len );
                break;
            }

            // Allocate packet in MPSC
            struct bm_rx_frame* frame = bm_rx_frame_alloc( &dev_data->enc_rx_mpsc, (uint16_t)len );
            if( !frame )
            {
                // Drop packet
                LOG_ERR( "Unable to allocate RX frame" );
                break;
            }

            // Write the frame descriptor
            bm_rx_frame_write_desc( frame, dev_idx, (uint16_t)len );

            // Copy the frame data
            if( ( tail + len ) <= buf_len )
            {
                // Single copy
                memcpy( frame->data, &buf[tail], len );
            }
            else
            {
                // Two copies required. Tail to end, beginning to head
                size_t len_1 = buf_len - tail;
                size_t len_2 = len - len_1;

                memcpy( frame->data,         &buf[tail], len_1 );
                memcpy( frame->data + len_1, &buf[0],    len_2 );
            }

            // Commit packet to queue and notify RX thread
            bm_rx_frame_commit( &dev_data->enc_rx_mpsc, frame );

            LOG_DBG("bm_serial_dma_cb - Len of Frame: %d | Port: %d", len ,dev_idx);

            k_sem_give( &dev_data->rx_int_sem );

            LOG_DBG( "Submitted encoded RX frame of len %d from dev %d", (int)len, (int)dev_idx );
            break;
        }

        case UART_RX_DISABLED:
        {
            LOG_ERR("Received DMA Disabled!");
            break;
        }

        case UART_RX_STOPPED:
        {
            LOG_ERR("Received DMA Stopped!");
            break;
        }

        // Not needed in circular buffer mode
        case UART_RX_BUF_RELEASED:
        case UART_RX_BUF_REQUEST:
        default:
        {
            break;
        }
    }
    irq_unlock(irq_lock_key);
}

static void bm_serial_rx_thread(void *arg1, void *unused1, void *unused2)
{
    size_t rx_frame_size;
    int i;
    uint8_t preamble_err = 0;

    /* Dev from message queue */
    uint8_t dev_idx;

    struct bm_rx_frame* frame;
    uint16_t frame_len;

    struct net_pkt *pkt;

    const struct device *dev;
    struct bm_stm32_hal_dev_data *dev_data;

    dev = (const struct device *)arg1;
	dev_data = dev->data;

	__ASSERT_NO_MSG(dev_data != NULL);

    LOG_DBG("BM Serial RX thread started");

    while (1)
    {
        // Clear preamble bits
        memset( dev_data->dec_rx_buf, 0, CONFIG_BM_STM32_HAL_PREAMBLE_LEN );

        // Wait for packets in MPSC queue
        k_sem_take( &dev_data->rx_int_sem, K_FOREVER );

        // Attempt to claim packet that should be waiting in queue
        frame = (struct bm_rx_frame *)mpsc_pbuf_claim(&dev_data->enc_rx_mpsc);
        if( !frame )
        {
            LOG_ERR( "RX thread semaphore notified, but no packet available" );
            continue;
        }

        // Get frame length and PHY index
        frame_len = frame->hdr.desc.data_len;
        dev_idx = (uint8_t)frame->hdr.desc.phy_idx;

        LOG_DBG( "Claimed frame of len: %d from dev %d", (int)frame_len, (int)dev_idx );

        // Copy payload
        rx_frame_size = frame_len;
        memcpy( dev_data->dec_rx_buf, frame->data, rx_frame_size );

        // Free claimed packet
        mpsc_pbuf_free( &dev_data->enc_rx_mpsc, (union mpsc_pbuf_generic*)frame );

        // Check pre-amble
        preamble_err = 0;
        for (i=0; i < CONFIG_BM_STM32_HAL_PREAMBLE_LEN; i++)
        {
            if ( dev_data->dec_rx_buf[i] != CONFIG_BM_STM32_HAL_PREAMBLE_VAL )
            {
                preamble_err = 1;
            } 
        }
        if (preamble_err)
        {
            LOG_ERR( "Invalid preamble in received frame" );
            continue;
        }

        // Remove preamble from length
        rx_frame_size -= CONFIG_BM_STM32_HAL_PREAMBLE_LEN;

        // Validate wire payload length after processing preamble
        if( rx_frame_size < BM_MIN_WIRE_FRAME_SIZE || rx_frame_size > BM_MAX_WIRE_FRAME_SIZE )
        {
            LOG_ERR( "Invalid frame size: %" PRIu32, rx_frame_size );
            continue;
        }
    
#ifdef BM_STM32_HAL_ENABLE_MANCHESTER_CODING

        // Decode MAC Header
#else
        // Copy MAC Header
#endif

        // Calculate pointers to message components
        // MAC Header -> IPv6 Header -> UDP Header -> Payload
        struct net_bm_hdr* mac_hdr          = (struct net_bm_hdr*)&dev_data->dec_rx_buf[CONFIG_BM_STM32_HAL_PREAMBLE_LEN];
        struct net_ipv6_hdr* ipv6_hdr_addr  = (struct net_ipv6_hdr*)((uint8_t*)mac_hdr + sizeof(struct net_bm_hdr));
        // struct net_udp_hdr *udp_hdr         = (struct net_udp_hdr*)((uint8_t*)ipv6_hdr_addr + sizeof(struct net_ipv6_hdr));
        // uint8_t* payload_ptr                = (uint8_t*)udp_hdr + sizeof(struct net_udp_hdr);
        // size_t payload_len                  = len - (payload_ptr - (uint8_t*)mac_hdr);

        size_t packet_len = rx_frame_size - sizeof(uint32_t);           // Size of frame minus preamble and MAC CRC32
        size_t mac_pl_length = packet_len - sizeof(struct net_bm_hdr);  // Size of packet minus BM MAC header

        // Validate MAC Header CRC16
        uint16_t calc_crc16 = crc16_ccitt( 0xffff, (uint8_t*)mac_hdr, sizeof(struct net_bm_hdr) - sizeof(uint16_t) );
        if (calc_crc16 != mac_hdr->crc)
        {
            LOG_ERR("MAC header CRC16 mismatch. Received: %" PRIu16 ", Computed: %" PRIu16, mac_hdr->crc, calc_crc16 );
            continue;
        }

        // Check supported ethertypes
        if( ntohs(mac_hdr->type) != NET_BM_PTYPE_IPV6 )
        {
            // Only IPv6 ethertypes are currently supported
            LOG_ERR("Dropping non-IPv6 message in RX thread: %d", (int)mac_hdr->type );
            continue;
        }

        // Check to see if address is multicast
        bool is_dest_mcast = net_ipv6_is_addr_mcast( (const struct in6_addr*)ipv6_hdr_addr->dst );
        if( !is_dest_mcast )
        {
            LOG_ERR( "Unicast not currently supported over Bristlemouth" );
            continue;
        }

        // Verify MAC Payload CRC32
        uint8_t* crc_offset = (uint8_t*)ipv6_hdr_addr + mac_pl_length;
        uint32_t crc32 = crc32_ieee( (uint8_t*)ipv6_hdr_addr, mac_pl_length );
        uint32_t rx_crc32 = (uint32_t)crc_offset[3] << 24UL |
                            (uint32_t)crc_offset[2] << 16UL |
                            (uint32_t)crc_offset[1] << 8UL |
                            (uint32_t)crc_offset[0];
        if (crc32 != rx_crc32)
        {
            LOG_ERR("CRC32 mismatch. Received: %" PRIu32 ", Computed: %" PRIu32, rx_crc32, crc32);
            continue;
        }

        // Modify IPv6 Source Address with remote port and local port information if address is link-local
        if( net_ipv6_is_ll_addr( (const struct in6_addr*)ipv6_hdr_addr->src ) )
        {
            // Bytes 8-15 contain EUI64 component, byte 6 = remote port, byte 7 = local port
            // Start values at 1 so that 0 values should never be possible
            ipv6_hdr_addr->src[6] = 1 + mac_hdr->src_port;
            ipv6_hdr_addr->src[7] = 1 + dev_idx;

            LOG_DBG( "Msg is link local. Modified src IP" );
        }
        else
        {
            // This message needs to be forwarded on to other devices as well
            // Re-transmit on all interfaces other than the one the message came in on
                
            // Allocate packet in MPSC
            uint16_t tx_frame_len = frame_len - CONFIG_BM_STM32_HAL_PREAMBLE_LEN - 4;
            struct bm_tx_frame* tx_frame = bm_tx_frame_alloc( &dev_data->tx_mpsc, (uint16_t)tx_frame_len );
            if( !tx_frame )
            {
                // Drop packet
                LOG_ERR( "Unable to allocate forwarding TX frame in MPSC" );
            }
            else
            {
                // Write the frame descriptor (phy_mask is dev_idx +1 to avoid 0 val which means all)
                bm_tx_frame_write_desc( tx_frame, dev_idx + 1, tx_frame_len );

                // Copy original data as-is
                memcpy( tx_frame->data, (uint8_t*)&dev_data->dec_rx_buf[CONFIG_BM_STM32_HAL_PREAMBLE_LEN], tx_frame_len );

                // Commit copied packet data to MPSC
                bm_tx_frame_commit( &dev_data->tx_mpsc, tx_frame );
                k_sem_give( &dev_data->tx_int_sem );

                LOG_DBG( "Forwarded frame of len %d", (int)tx_frame_len );
            }
        }

        // Create packet
        pkt = net_pkt_rx_alloc_with_buffer( dev_data->iface, packet_len, AF_UNSPEC, 0, K_MSEC(100));
        if (!pkt) {
            LOG_ERR("Failed to obtain RX packet buffer");
            continue;
        }

        if (net_pkt_write(pkt, (uint8_t*)mac_hdr, packet_len)) {
            LOG_ERR("Unable to write frame into the pkt");
            net_pkt_unref(pkt);
            pkt = NULL;
            continue;
        }

        int res = net_recv_data(dev_data->iface, pkt);
        if (res < 0) {
            LOG_ERR("Error receiving data: %d", res );
            net_pkt_unref(pkt);
            continue;
        }

        LOG_DBG( "Successfully passed packet to upper layer" );
    }
}

int process_tx_packet( struct bm_stm32_hal_dev_data *dev_data, struct bm_tx_frame* frame )
{
    int ret = 0;

    uint16_t len = (uint16_t)frame->hdr.desc.data_len;
    LOG_DBG( "TX process packet of len: %d", (int)len );

    // Calculate pointers to message components
    // MAC Header -> IPv6 Header -> UDP Header -> Payload
    struct net_bm_hdr* mac_hdr          = (struct net_bm_hdr*)frame->data;
    struct net_ipv6_hdr* ipv6_hdr_addr  = (struct net_ipv6_hdr*)((uint8_t*)mac_hdr + sizeof(struct net_bm_hdr));
    // struct net_udp_hdr *udp_hdr         = (struct net_udp_hdr*)((uint8_t*)ipv6_hdr_addr + sizeof(struct net_ipv6_hdr));
    // uint8_t* payload_ptr                = (uint8_t*)udp_hdr + sizeof(struct net_udp_hdr);
    // size_t payload_len                  = len - (payload_ptr - (uint8_t*)mac_hdr);
    
    size_t mac_payload_len              = len - (sizeof(struct net_bm_hdr));

    // Check supported ethertypes
    if( ntohs(mac_hdr->type) != NET_BM_PTYPE_IPV6 )
    {
        // Only IPv6 ethertypes are currently supported
        LOG_ERR("Dropping non-IPv6 message in TX thread: %d", (int)mac_hdr->type );
        ret = -EINVAL;
        goto error;
    }

    // Check to see if address is multicast
    bool is_dest_mcast = net_ipv6_is_addr_mcast( (const struct in6_addr*)ipv6_hdr_addr->dst );
    if( !is_dest_mcast )
    {
        LOG_ERR( "Unicast not currently supported over Bristlemouth" );
        ret = -EINVAL;
        goto error;
    }

    // Get forwarding/routing excluded port
    // TODO: Make proper use of mask to exclude multiple ports for unicast routing
    bool is_routed = ( frame->hdr.desc.phy_mask != 0 );
    uint8_t src_dev_idx = frame->hdr.desc.phy_mask - 1;

    // Calculate CRC32 for IPv6 + UDP + Payload (The entire MAC payload)
    uint32_t mac_pl_crc32 = crc32_ieee( (uint8_t*)ipv6_hdr_addr, len - sizeof(struct net_bm_hdr) );

    // Collect the list of ports that need to be sent on
    struct bm_tx_dma_buf* first_dev = NULL;
    for( int n = 0; n < CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT; ++n )
    {
        // When routing messages, skip sending to the device index that received the message
        if( is_routed && ( n == src_dev_idx ) )
        {
            LOG_DBG( "Skipping transmission of frame on port %d", n );
            continue;
        }

        // Write preamble
        memset( dev_data->tx_dma_buf[n].data, CONFIG_BM_STM32_HAL_PREAMBLE_VAL, CONFIG_BM_STM32_HAL_PREAMBLE_LEN );

        // Modify header
        mac_hdr->src_port = n;

        // Calculate CRC16 for MAC header info
        mac_hdr->crc = crc16_ccitt( 0xffff, (uint8_t*)mac_hdr, sizeof(struct net_bm_hdr) - sizeof(uint16_t) );

#ifdef BM_STM32_HAL_ENABLE_MANCHESTER_CODING
        // TODO: Encode
        // #ifdef BM_STM32_HAL_ENABLE_MANCHESTER_CODING
//         LOG_DBG("Encoding");
//         uint16_t encoded_val;

//         // Encode payload
//         for ( i=0; i < frame_length; i++)
//         {
//             encoded_val = BM_MAN_ENCODE_TABLE[frame_addr[i]];
//             tx_enc_buf[tx_buf_ctr++] = (uint8_t) (encoded_val & 0xFF);
//             tx_enc_buf[tx_buf_ctr++] = (uint8_t) ((encoded_val >> 8) & 0xFF);
//         }
// #else
//         // Copy payload
//         memcpy( tx_enc_buf + tx_buf_ctr, frame_addr, frame_length );
//         tx_buf_ctr += frame_length;
// #endif
#else
        // Write MAC header to DMA buffer
        memcpy( dev_data->tx_dma_buf[n].data + CONFIG_BM_STM32_HAL_PREAMBLE_LEN, (void*)mac_hdr, sizeof(struct net_bm_hdr) );
#endif
        // data + sizeof(struct net_bm_hdr), mac_payload_len
        
        // Encode payload directly into first interface's DMA TX buffer
        if( !first_dev )
        {
#ifdef BM_STM32_HAL_ENABLE_MANCHESTER_CODING
            // TODO: Encode
#else
            // Get data destination
            uint8_t* data_dst = &dev_data->tx_dma_buf[n].data[ CONFIG_BM_STM32_HAL_PREAMBLE_LEN + sizeof(struct net_bm_hdr) ];

            // Copy payload into DMA buffer after MAC header
            memcpy( data_dst, ipv6_hdr_addr, mac_payload_len );

            // Write CRC32 for MAC Payload after payload
            memcpy( data_dst + mac_payload_len, &mac_pl_crc32, sizeof(mac_pl_crc32) );

            // Set the first dev index so the other interfaces can directly copy it's data
            first_dev = &dev_data->tx_dma_buf[n];
#endif
        }
        else
        {
            uint8_t* src_addr = &first_dev->data[ CONFIG_BM_STM32_HAL_PREAMBLE_LEN + sizeof(struct net_bm_hdr) ];
            uint8_t* data_dst = &dev_data->tx_dma_buf[n].data[ CONFIG_BM_STM32_HAL_PREAMBLE_LEN + sizeof(struct net_bm_hdr) ];

            // Copy payload from first interface
            memcpy( data_dst, src_addr, mac_payload_len );

            // Write CRC32 for MAC Payload after payload
            memcpy( data_dst + mac_payload_len, &mac_pl_crc32, sizeof(mac_pl_crc32) );
        }

        // Set total length of DMA buffer
#ifdef BM_STM32_HAL_ENABLE_MANCHESTER_CODING
        dev_data->tx_dma_buf[n].len = CONFIG_BM_STM32_HAL_PREAMBLE_LEN + (2 * len) + (2 * sizeof(uint32_t));
#else
        dev_data->tx_dma_buf[n].len = CONFIG_BM_STM32_HAL_PREAMBLE_LEN + len + sizeof(uint32_t);
#endif
    }

    // Wait for all available outbound semaphores
    for( int n = 0; n < CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT; ++n )
    {
        // When routing messages, skip sending to the device index that received the message
        if( is_routed && ( n == src_dev_idx ) )
        {
            continue;
        }

        k_sem_take( &dev_ctx[n].sem, K_FOREVER );
    }

    // Start all outbound DMA transfers
    for( int n = 0; n < CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT; ++n )
    {
        // When routing messages, skip sending to the device index that received the message
        if( is_routed && ( n == src_dev_idx ) )
        {
            continue;
        }

        LOG_DBG("process_tx_packet - Port Num: %d | Len: %d", n, dev_data->tx_dma_buf[n].len);

        // TODO: Does the timeout value here even do anything?
        if( uart_tx( dev_ctx[n].serial_dev, dev_data->tx_dma_buf[n].data, dev_data->tx_dma_buf[n].len, 20 * USEC_PER_MSEC ) )
        {
            LOG_ERR( "Error transmitting frame on dev[%d]", n );
        }
    }

error:
    return ret;
}

static void bm_serial_tx_thread( void* arg1, void* arg2, void* arg3 )
{
    const struct device *dev;
    struct bm_stm32_hal_dev_data *dev_data;

    dev = (const struct device *)arg1;
	dev_data = dev->data;

    int ret;

    struct bm_tx_frame* frame;

    while (1) 
    {
        k_sem_take(&dev_data->tx_int_sem, K_FOREVER );

        // Attempt to claim packet that should be waiting in queue
        frame = (struct bm_tx_frame *)mpsc_pbuf_claim(&dev_data->tx_mpsc);
        if( !frame )
        {
            LOG_ERR( "TX thread semaphore notified, but no packet available" );
            continue;
        }

        // Process outbound frame
        ret = process_tx_packet( dev_data, frame );
        LOG_DBG( "Processed packet from TX MPSC with ret=%d", ret );

        // Free MPSC packet, regardless of result
        mpsc_pbuf_free( &dev_data->tx_mpsc, (union mpsc_pbuf_generic*)frame );
        LOG_DBG( "Freed %dB packet from TX MPSC", (int)frame->hdr.desc.data_len );

        if( ret )
        {
            LOG_ERR( "Failed to process frame: Err=%d", ret );
            continue;
        }
    }
}

static inline uint32_t rx_mpsc_get_wlen(const union mpsc_pbuf_generic *item)
{
	const union bm_rx_frame_generic *generic_msg = (const union bm_rx_frame_generic *)item;

    const struct bm_rx_frame *frame = (const struct bm_rx_frame*)generic_msg;

    return bm_rx_frame_get_total_wlen(frame->hdr.desc);
}

static inline uint32_t tx_mpsc_get_wlen(const union mpsc_pbuf_generic *item)
{
	const union bm_tx_frame_generic *generic_msg = (const union bm_tx_frame_generic *)item;

    const struct bm_tx_frame *frame = (const struct bm_tx_frame*)generic_msg;

    return bm_tx_frame_get_total_wlen(frame->hdr.desc);
}

static int bm_initialize(const struct device *dev)
{
    int ret = 0;
    struct bm_stm32_hal_dev_data *dev_data;

    __ASSERT_NO_MSG(dev != NULL);

	dev_data = dev->data;

    __ASSERT_NO_MSG(dev_data != NULL);

    // TODO: Setup MAC from uuid using murmur_32 hash or random
#if defined(CONFIG_BM_STM32_HAL_RANDOM_MAC)
	generate_mac(dev_data->mac_addr);
#endif

    static const struct device* _dev;
    uint8_t counter = 0;
    uint32_t interframe_delay_us;
    uint32_t* baud_rate_addr;

    // Setup TX MPSC buffer
    memset(dev_data->tx_mpsc_buffer, 0, sizeof(dev_data->tx_mpsc_buffer));
    dev_data->tx_mpsc_cfg = (struct mpsc_pbuf_buffer_config){
        .buf            = dev_data->tx_mpsc_buffer,
        .size           = ARRAY_SIZE(dev_data->tx_mpsc_buffer),
        .notify_drop    = NULL,
        .get_wlen       = tx_mpsc_get_wlen,
        .flags          = 0
    };
    mpsc_pbuf_init( &dev_data->tx_mpsc, &dev_data->tx_mpsc_cfg );
    k_sem_init( &dev_data->tx_int_sem, 0, 20 ); // TODO: Make this configurable

    // Setup RX MPSC buffer
    memset(dev_data->enc_rx_mpsc_buffer, 0, sizeof(dev_data->enc_rx_mpsc_buffer));
    dev_data->enc_rx_mpsc_drop_cnt = 0;
    dev_data->enc_rx_mpsc_cfg = (struct mpsc_pbuf_buffer_config){
        .buf            = dev_data->enc_rx_mpsc_buffer,
        .size           = ARRAY_SIZE(dev_data->enc_rx_mpsc_buffer),
        .notify_drop    = NULL,
        .get_wlen       = rx_mpsc_get_wlen,
        .flags          = 0
    };
    mpsc_pbuf_init( &dev_data->enc_rx_mpsc, &dev_data->enc_rx_mpsc_cfg );
    k_sem_init( &dev_data->rx_int_sem, 0, 50 ); // TODO: Make this configurable

    // Initialize BM UART devices
    // TODO: Succinctly handle this in a loop for available devices or at least extract into method
    _dev = device_get_binding(CONFIG_BM_STM32_HAL_SERIAL_DEV_NAME_0);
    if (_dev != NULL) 
    {
        dev_ctx[0].serial_dev = _dev;

        baud_rate_addr = (uint32_t*) (_dev->data);
        interframe_delay_us = ((1000000000UL / *baud_rate_addr) * 2 * 10)/1000;
        dev_ctx[0].interframe_delay_us = interframe_delay_us;

        k_sem_init(( struct k_sem* ) &dev_ctx[0].sem, 1, 1);
        k_timer_init(( struct k_timer* ) &dev_ctx[0].timer, tx_dma_timer_handler, NULL);
        uart_callback_set(_dev, bm_serial_dma_cb, dev_data );

        /* Enable RX DMA (should be in circular buffer mode) */
        uart_rx_enable(( const struct device* )dev_ctx[0].serial_dev, dev_ctx[0].buf, sizeof(dev_ctx[0].buf), -1);
        counter++;
    }

    _dev = device_get_binding(CONFIG_BM_STM32_HAL_SERIAL_DEV_NAME_1);
    if (_dev != NULL) 
    {
        dev_ctx[1].serial_dev = _dev;

        baud_rate_addr = (uint32_t*) (_dev->data);
        interframe_delay_us = ((1000000000UL / *baud_rate_addr) * 2 * 10)/1000;
        dev_ctx[1].interframe_delay_us = interframe_delay_us;

        k_sem_init(( struct k_sem* ) &dev_ctx[1].sem, 1, 1);
        k_timer_init(( struct k_timer* ) &dev_ctx[1].timer, tx_dma_timer_handler, NULL);
        uart_callback_set(_dev, bm_serial_dma_cb, dev_data);

        /* Enable RX DMA (should be in circular buffer mode) */
        uart_rx_enable(( const struct device* ) dev_ctx[1].serial_dev, dev_ctx[1].buf, sizeof(dev_ctx[1].buf), -1);

        counter++;
    }

    _dev = device_get_binding(CONFIG_BM_STM32_HAL_SERIAL_DEV_NAME_2);
    if (_dev != NULL) 
    {
        dev_ctx[2].serial_dev = _dev;

        baud_rate_addr = (uint32_t*) (_dev->data);
        interframe_delay_us = ((1000000000UL / *baud_rate_addr) * 2 * 10)/1000;
        dev_ctx[2].interframe_delay_us = interframe_delay_us;

        k_sem_init(( struct k_sem* ) &dev_ctx[2].sem, 1, 1);
        k_timer_init(( struct k_timer* ) &dev_ctx[2].timer, tx_dma_timer_handler, NULL);
        uart_callback_set(_dev, bm_serial_dma_cb, dev_data);

        /* Enable RX DMA (should be in circular buffer mode) */
        uart_rx_enable(( const struct device* ) dev_ctx[2].serial_dev, dev_ctx[2].buf, sizeof(dev_ctx[2].buf), -1);

        counter++;
    }

    k_thread_create( &dev_data->rx_thread, dev_data->rx_thread_stack,
            K_THREAD_STACK_SIZEOF(dev_data->rx_thread_stack),
            bm_serial_rx_thread, (void*)dev, NULL, NULL, 
            K_PRIO_COOP(10), 
            0, K_NO_WAIT );
    k_thread_name_set(&dev_data->rx_thread, "bm_l2_rx");

    k_thread_create( &dev_data->tx_thread, dev_data->tx_thread_stack,
            K_THREAD_STACK_SIZEOF(dev_data->tx_thread_stack),
            bm_serial_tx_thread, (void*)dev, NULL, NULL, 
            K_PRIO_COOP(10), 
            0, K_NO_WAIT );
    k_thread_name_set(&dev_data->tx_thread, "bm_l2_tx");

    // TODO: Error handling
    return ret;
}


static void bm_iface_init(struct net_if *iface)
{
	const struct device *dev;
	struct bm_stm32_hal_dev_data *dev_data;

	__ASSERT_NO_MSG(iface != NULL);

	dev = net_if_get_device(iface);
	__ASSERT_NO_MSG(dev != NULL);

	dev_data = dev->data;
	__ASSERT_NO_MSG(dev_data != NULL);

    if (dev_data->iface == NULL) {
		dev_data->iface = iface;
	}
    
    // TODO: Set MAC lower-3 bytes based on some unique info from ST 96-bit UID

	/* Register Ethernet MAC Address with the upper layer */
	net_if_set_link_addr(iface, dev_data->mac_addr,
			     sizeof(dev_data->mac_addr),
			     NET_LINK_BRISTLEMOUTH);

	bristlemouth_init(iface);

	net_if_flag_set(iface, NET_IF_NO_AUTO_START);

    // Bring the network interface up immediately
    net_bm_carrier_on( iface );
}

static enum bristlemouth_hw_caps bm_stm32_hal_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

    // Default to full duplex
    return BRISTLEMOUTH_DUPLEX_SET;
}

static int bm_stm32_hal_set_config(const struct device *dev,
				    enum bristlemouth_config_type type,
				    const struct bristlemouth_config *config)
{
	int ret = -ENOTSUP;
	struct bm_stm32_hal_dev_data *dev_data;

	dev_data = dev->data;

	switch (type) {
	case BRISTLEMOUTH_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(dev_data->mac_addr, config->mac_address.addr, 6);

		net_if_set_link_addr(dev_data->iface, 
                    dev_data->mac_addr,
				    sizeof(dev_data->mac_addr),
				    NET_LINK_BRISTLEMOUTH);
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

// static struct net_pkt *bm_rx(const struct device *dev)
// {
//     struct bm_stm32_hal_dev_data *dev_data;
//     struct net_pkt *pkt;
//     size_t total_len;

//     (void)pkt;
//     (void)total_len;

//     dev_data = dev->data;
//     __ASSERT_NO_MSG(dev_data != NULL);

//     // TODO: Move logic that is currently embedded in rx_thread to here
//     // TODO: Add interface fragment to packet

//     return NULL;
// }

// Queues incoming packet from upper-L2 layer into MPSC buffer for consumption by TX thread
static int bm_tx_queue( const struct device *dev, struct net_pkt* pkt )
{
    struct bm_stm32_hal_dev_data *dev_data = dev->data;
    int ret = 0;
    size_t total_len;

	__ASSERT_NO_MSG(pkt != NULL);
	__ASSERT_NO_MSG(pkt->frags != NULL);
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(dev_data != NULL);

    k_mutex_lock( &dev_data->tx_mutex, K_FOREVER );

    // Get total length of packet
    total_len = net_pkt_get_len(pkt);
	if (total_len > NET_BM_MAX_FRAME_SIZE) {
		LOG_ERR("TX packet too big");
		ret = -EIO;
		goto error;
	}

    // Allocate packet in MPSC
    struct bm_tx_frame* frame = bm_tx_frame_alloc( &dev_data->tx_mpsc, (uint16_t)total_len );
    if( !frame )
    {
        // Drop packet
        LOG_ERR( "Unable to allocate TX frame in MPSC" );
        ret = -ENOSPC;
		goto error;
    }

    // Write the frame descriptor
    bm_tx_frame_write_desc( frame, 0, (uint16_t)total_len );

    // Attempt to read data into allocated buffer
    if (net_pkt_read(pkt, frame->data, total_len)) {
        LOG_ERR("Couldn't read packet");
		ret = -ENOBUFS;
		goto error;
	}

    // Commit copied packet data to MPSC
    bm_tx_frame_commit( &dev_data->tx_mpsc, frame );
    k_sem_give( &dev_data->tx_int_sem );

error:
    k_mutex_unlock( &dev_data->tx_mutex );

    return ret;
}

static const struct bristlemouth_api bm_api = {
	.iface_api.init     = bm_iface_init,
	.get_capabilities   = bm_stm32_hal_get_capabilities,
	.set_config         = bm_stm32_hal_set_config,
    .send               = bm_tx_queue,
};

static struct bm_stm32_hal_dev_data bm0_data = {
	.mac_addr = {
		SOFAR_OUI_B0,
		SOFAR_OUI_B1,
		SOFAR_OUI_B2,
#if !defined(CONFIG_BM_STM32_HAL_RANDOM_MAC)
		CONFIG_BM_STM32_HAL_MAC3,
		CONFIG_BM_STM32_HAL_MAC4,
		CONFIG_BM_STM32_HAL_MAC5
#endif
	},
};

BM_NET_DEVICE_INIT( bristlemouth_stm32_hal, 
    "bm0", 
    bm_initialize,
    NULL, 
    &bm0_data, 
    NULL,
	CONFIG_BM_INIT_PRIORITY, 
    &bm_api, 
    BM_STM32_HAL_MTU );


// TODO:
// x - Set up initial link-local address (or does zephyr do this automatically in ipv6 layer?)
// x - IPv6 ND exists in zephyr IP layer. Disable
// x - Set up well known multicast addresses
// x - Pare down header info from bm_serial
// x - Add support for introspecting MAC header to see if message needs forwarding
// x - Add support for tagging packets in RX thread with the interface they came from
// Add support for routing packets TX to only the interfaces they need to go to
// Add support for forwarding multicast based on the destination addr (ff02::1 vs ff03::1)

// Notes on order of startup:
// - Driver init comes first, then L2 init:

// CONFIG_BM_INIT_PRIORITY = 80
// net_init() -> init_rx_queues() -> net_if_init() -> init_iface(struct net_if *iface) ->  iface_api->init

// CONFIG_NET_INIT_PRIO = 90
// SYS_INIT(net_init, POST_KERNEL, CONFIG_NET_INIT_PRIO);

// 1. bm_initialize(): rx/tx threads created, UART interfaces set up and enabled. bm_frame_t can now be tx/rx'd
// 2. bm_iface_init(): MAC address is set. Then calls: 
// 3.   bristlemouth_init(): Upper L2 layer initialization. Sets flags and inits work item for carrier on/off
// 4.   net_bm_carrier_on() is called, bringing the interface up automatically at the end of initialization
// 5. Network interface is now usable

// <!-- static inline void l3_init(void)
// {
// 	net_icmpv4_init();
// 	net_icmpv6_init();
// 	net_ipv6_init();

// 	net_ipv4_autoconf_init();

// 	if (IS_ENABLED(CONFIG_NET_UDP) ||
// 	    IS_ENABLED(CONFIG_NET_TCP) ||
// 	    IS_ENABLED(CONFIG_NET_SOCKETS_PACKET) ||
// 	    IS_ENABLED(CONFIG_NET_SOCKETS_CAN)) {
// 		net_conn_init();
// 	}

// 	net_tcp_init();

// 	net_route_init();

// 	NET_DBG("Network L3 init done");
// }

// static inline int services_init(void)
// {
// 	int status;

// 	status = net_dhcpv4_init();
// 	if (status) {
// 		return status;
// 	}

// 	dns_init_resolver();
// 	websocket_init();

// 	net_coap_init();

// 	net_shell_init();

// 	return status;
// }

// static int net_init(const struct device *unused)
// {
// 	net_hostname_init();

// 	NET_DBG("Priority %d", CONFIG_NET_INIT_PRIO);

// 	net_pkt_init();

// 	net_context_init();

// 	l3_init();

// 	net_mgmt_event_init();

// 	init_rx_queues();

// 	return services_init();
// } -->


// Helpers:

// Iterate/Read frags:
	// size_t bytes = 0;
    // struct net_buf *buf = pkt->frags;
    // int i = 0;
	// while (buf) {

    //     uint8_t* d = buf->data;
    //     LOG_DBG("Frag %d: size=%d", i, buf->len);
    //     LOG_DBG("First values of frag: %d %d %d %d %d %d %d %d %d %d %d %d %d %d", 
    //         d[0], d[1], d[2], d[3], d[4], d[5],
    //         d[6], d[7], d[8], d[9], d[10], d[11],
    //         d[12], d[13]);

	// 	bytes += buf->len;
	// 	buf = buf->frags;
    //     i++;
	// }