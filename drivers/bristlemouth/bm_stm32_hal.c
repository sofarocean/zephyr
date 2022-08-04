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
#define LOG_LEVEL CONFIG_BRISTLEMOUTH_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/crc.h>
#include <zephyr/random/rand32.h>


#include "bm_stm32_hal_priv.h"
#include "manchester.h"
#include "murmur_hash.h"

#define SOFAR_OUI_B0 0xA0
#define SOFAR_OUI_B1 0x5E
#define SOFAR_OUI_B2 0xA5

static volatile bm_ctx_t dev_ctx[CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT];

static struct k_thread tx_thread_data;
static struct k_thread rx_thread_data;

/* Buffer for received frame payloads */
static uint8_t rx_payload_buf[CONFIG_BM_STM32_HAL_NUM_FRAMES * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE];
static uint8_t rx_payload_idx = 0;
static uint8_t rx_dec_buf[2*(CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE) + CONFIG_BM_STM32_HAL_PREAMBLE_LEN];

/* Buffer for TX frame payloads */
static uint8_t tx_payload_buf[CONFIG_BM_STM32_HAL_NUM_FRAMES * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE];
static uint8_t tx_payload_idx = 0;
static uint8_t tx_enc_buf[2*(CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE) + CONFIG_BM_STM32_HAL_PREAMBLE_LEN];

/* RX/TX Threads and associated message Queues */
K_THREAD_STACK_DEFINE(bm_tx_stack, CONFIG_BM_STM32_HAL_TASK_STACK_SIZE);
K_MSGQ_DEFINE(tx_queue, sizeof(bm_msg_t), CONFIG_BM_STM32_HAL_NUM_FRAMES, 4);
K_THREAD_STACK_DEFINE(bm_rx_stack, CONFIG_BM_STM32_HAL_TASK_STACK_SIZE);
K_MSGQ_DEFINE(rx_queue, sizeof(bm_msg_t), CONFIG_BM_STM32_HAL_NUM_FRAMES, 4);

K_MSGQ_DEFINE(encoded_rx_queue, sizeof(uint8_t), CONFIG_BM_STM32_HAL_NUM_FRAMES, 4);

/* Semaphore for ISR and RX_Task to signal availability of Decoded Frame */
K_SEM_DEFINE(dma_idx_sem, 1, 1);

static void linear_memcpy(uint8_t* dest, uint8_t* src, size_t n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        dest[i] = src[i];
    }
}

static void tx_dma_timer_handler(struct k_timer *tmr)
{
    int i;

    for (i = 0; i < 3; i++)
    {
        if ( tmr == &dev_ctx[i].timer)
        {
            k_sem_give((struct k_sem*) &dev_ctx[i].sem);
            break;
        }
    }
}

/**
 * BM Serial RX Thread
 */
static void bm_serial_rx_thread(void *arg1, void *unused1, void *unused2)
{
    size_t rx_frame_size;
    int i;
    uint8_t preamble_err = 0;

    /* Dev from message queue */
    uint8_t dev_idx;

    struct net_pkt *pkt;

    const struct device *dev;
    struct bm_stm32_hal_dev_data *dev_data;

    dev = (const struct device *)arg1;
	dev_data = dev->data;

	__ASSERT_NO_MSG(dev_data != NULL);

    LOG_DBG("BM Serial RX thread started");

    while (1)
    {
        k_msgq_get(&encoded_rx_queue, &dev_idx, K_FOREVER);

        /* Copy over DMA data and then release semaphore for DMA cb */
        k_sem_take(&dma_idx_sem , K_FOREVER);
        
        rx_frame_size = dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].read_buf_idx].length;
        memcpy( rx_dec_buf, (const uint8_t * ) dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].read_buf_idx].buf, rx_frame_size);
        
        k_sem_give(&dma_idx_sem);

        /* Check Preamble */ 
        preamble_err = 0;
        for (i=0; i < CONFIG_BM_STM32_HAL_PREAMBLE_LEN; i++)
        {
            if ( rx_dec_buf[i] != CONFIG_BM_STM32_HAL_PREAMBLE_VAL )
            {
                preamble_err = 1;
            } 
        }

        /* If 4 Preamble bytes are not found in begining of packet, then wait for next packet */
        if (preamble_err)
        {
            LOG_ERR("Missing or Corrupt Preamble");
            continue;
        }

        // Remove preamble from length
        rx_frame_size -= CONFIG_BM_STM32_HAL_PREAMBLE_LEN;

#ifdef BM_STM32_HAL_ENABLE_MANCHESTER_CODING

        // Decode payload
#else
        // Copy payload
#endif

        // Get data offset
        uint8_t* payload_buf = &rx_dec_buf[CONFIG_BM_STM32_HAL_PREAMBLE_LEN];
        uint8_t* crc_offset = &payload_buf[ rx_frame_size - sizeof(uint32_t)];

        /* Verify CRC16 (Bristlemouth Packet = header + payload)*/
        uint32_t crc32 = crc32_ieee( payload_buf, rx_frame_size - sizeof(uint32_t) );
        uint32_t rx_crc32 = (uint32_t)crc_offset[3] << 24UL |
                            (uint32_t)crc_offset[2] << 16UL |
                            (uint32_t)crc_offset[1] << 8UL |
                            (uint32_t)crc_offset[0];

        if (crc32 != rx_crc32)
        {
            LOG_ERR("CRC32 mismatch. Received: %" PRIu32 ", Computed: %" PRIu32, rx_crc32, crc32);
            continue;
        }

        rx_frame_size -= sizeof(uint32_t);

        // Create packet
        pkt = net_pkt_rx_alloc_with_buffer( dev_data->iface, rx_frame_size, AF_UNSPEC, 0, K_MSEC(100));
        if (!pkt) {
            LOG_ERR("Failed to obtain RX packet buffer");
            continue;
        }

        if (net_pkt_write(pkt, payload_buf, rx_frame_size)) {
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

        LOG_INF( "Successfully passed packet to upper layer" );
    }
}

static void bm_serial_dma_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    int i;
    size_t old_pos;
    size_t pos;
    uint8_t * buf;
    size_t buf_len = 0;
    uint8_t dev_idx;
    int retval;

    switch (evt->type)
    {
        case UART_TX_DONE:
        case UART_TX_ABORTED:
            for (i = 0; i < CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT; i++)
            {
                if ( dev == dev_ctx[i].serial_dev)
                {
                    k_timer_start((struct k_timer*) &dev_ctx[i].timer, K_USEC(dev_ctx[i].interframe_delay_us), K_NO_WAIT);
                }
            }
            break;
        case UART_RX_RDY:
            pos = evt->data.rx.len;
            old_pos = evt->data.rx.offset;
            buf = evt->data.rx.buf;

            /* Added this in case the buffer lengths are different between devices? */
            for (i = 0; i < CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT; i++)
            {
                if ( dev == dev_ctx[i].serial_dev)
                {
                    dev_idx = i;
                    buf_len = sizeof(dev_ctx[i].buf);
                }
            }

            /* This shouldn't happen */
            if (buf_len == 0)
            {
                LOG_ERR("RX Buf provided to BM UART driver has length of 0?\n");
            }

            if (pos != old_pos) 
            {
                /* Current position is over previous one */
                if (pos > old_pos) 
                {
                    /* We are in "linear" mode
                          Process data directly by subtracting "pointers" */
                    linear_memcpy(( uint8_t* )dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].write_buf_idx].buf, &buf[old_pos], pos-old_pos);
                    dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].write_buf_idx].length = (pos - old_pos);
                }
                else
                {
                    
                    /* We are in "overflow" mode
                       First process data to the end of buffer */

                    /* Check and continue with beginning of buffer */
                    linear_memcpy(( uint8_t* ) dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].write_buf_idx].buf, &buf[old_pos], buf_len-old_pos);
                    dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].write_buf_idx].length = (buf_len-old_pos);
                    if (pos) 
                    {
                        linear_memcpy(( uint8_t* ) &dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].write_buf_idx].buf[buf_len-old_pos], &buf[0], pos);
                        dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].write_buf_idx].length += pos;
                    }
                }

                /* Before flipping read/write indices, try to grab semaphore */
                if (k_sem_take(&dma_idx_sem , K_NO_WAIT) != 0)
                {
                    LOG_ERR("Can't take DMA Index semaphore");
                    break;
                }
                                
                /* Flip idx of read and write buffers, and release semaphore */
                dev_ctx[dev_idx].read_buf_idx = dev_ctx[dev_idx].write_buf_idx;
                dev_ctx[dev_idx].write_buf_idx = 1 - dev_ctx[dev_idx].write_buf_idx;
                k_sem_give(&dma_idx_sem);

                retval = k_msgq_put(&encoded_rx_queue, &dev_idx, K_FOREVER);
            }
            break;
        case UART_RX_DISABLED:
            LOG_ERR("Received DMA Disabled!");
            break;
        case UART_RX_STOPPED:
            LOG_ERR("Received DMA Stopped!");
            break;
        case UART_RX_BUF_RELEASED:
            /* Not needed in circular buffer mode */
        case UART_RX_BUF_REQUEST:
            /* Not needed in circular buffer mode */
        default:
            break;
    }
}

/**
 * BM Serial TX Thread
 */
static void bm_serial_tx_thread(void)
{
    bm_msg_t msg;
    uint8_t* frame_addr;
    uint16_t frame_length;
    uint8_t n;
    uint16_t tx_buf_ctr = 0;

    while (1) 
    {
        k_msgq_get(&tx_queue, &msg, K_FOREVER);

        frame_addr = msg.frame_addr;
        frame_length = msg.frame_length;

        if (frame_length > sizeof(tx_enc_buf))
        {
            LOG_ERR("TX Msg size too large to send. Ignoring");
            continue;
        }

        // TODO: Determine where this message is bound for by accessing the MAC/IP info
            // Get MAC DST
            // Get IPv6/etc DST based on ethertype

        // Write preamble
        for ( tx_buf_ctr = 0; tx_buf_ctr < CONFIG_BM_STM32_HAL_PREAMBLE_LEN; tx_buf_ctr++)
        {
            tx_enc_buf[tx_buf_ctr] = CONFIG_BM_STM32_HAL_PREAMBLE_VAL;
        }

#ifdef BM_STM32_HAL_ENABLE_MANCHESTER_CODING
        LOG_INF("Encoding");
        uint16_t encoded_val;

        // Encode payload
        for ( i=0; i < frame_length; i++)
        {
            encoded_val = BM_MAN_ENCODE_TABLE[frame_addr[i]];
            tx_enc_buf[tx_buf_ctr++] = (uint8_t) (encoded_val & 0xFF);
            tx_enc_buf[tx_buf_ctr++] = (uint8_t) ((encoded_val >> 8) & 0xFF);
        }
#else
        // Copy payload
        memcpy( tx_enc_buf + tx_buf_ctr, frame_addr, frame_length );
        tx_buf_ctr += frame_length;
#endif

        for ( n=0; n < CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT; n++)
        {
            if (dev_ctx[n].serial_dev != NULL)
            {
                /* Try grabbing semaphore  (wait forever if not available) */
                k_sem_take(( struct k_sem* ) &dev_ctx[n].sem, K_FOREVER);
            }
        }

        for ( n=0; n < CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT; n++)
        {
            if (dev_ctx[n].serial_dev != NULL)
            {
                /* TODO: Determine what this timeout is doing and whether its blocking/delaying 
                   any critical functionality */ 
                if( uart_tx(dev_ctx[n].serial_dev, tx_enc_buf, tx_buf_ctr, 10 * USEC_PER_MSEC) )
                {
                    LOG_ERR( "Couldn't send" );
                }
            }
        }
    }
}

struct k_msgq* bm_serial_get_rx_msgq_handler(void)
{
    /* Simple getter for allowing ieee802154_bm_serial.c to listen to message queue */
    return &rx_queue;
}

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

    _dev = device_get_binding(CONFIG_BM_STM32_HAL_SERIAL_DEV_NAME_0);
    if (_dev != NULL) 
    {
        dev_ctx[0].serial_dev = _dev;

        baud_rate_addr = (uint32_t*) (_dev->data);
        interframe_delay_us = ((1000000000UL / *baud_rate_addr) * 2 * 10)/1000;
        dev_ctx[0].interframe_delay_us = interframe_delay_us;

        k_sem_init(( struct k_sem* ) &dev_ctx[0].sem, 1, 1);
        k_timer_init(( struct k_timer* ) &dev_ctx[0].timer, tx_dma_timer_handler, NULL);
        uart_callback_set(_dev, bm_serial_dma_cb, NULL);

        /* Enable RX DMA (should be in circular buffer mode) */
        uart_rx_enable(( const struct device* ) dev_ctx[0].serial_dev, ( uint8_t* ) dev_ctx[0].buf, sizeof(dev_ctx[0].buf), -1);

        dev_ctx[0].write_buf_idx = 0;
        dev_ctx[0].read_buf_idx = 1;

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
        uart_callback_set(_dev, bm_serial_dma_cb, NULL);

        /* Enable RX DMA (should be in circular buffer mode) */
        uart_rx_enable(( const struct device* ) dev_ctx[1].serial_dev, ( uint8_t* ) dev_ctx[1].buf, sizeof(dev_ctx[1].buf), 0);

        dev_ctx[1].write_buf_idx = 0;
        dev_ctx[1].read_buf_idx = 1;

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
        uart_callback_set(_dev, bm_serial_dma_cb, NULL);

        /* Enable RX DMA (should be in circular buffer mode) */
        uart_rx_enable(( const struct device* ) dev_ctx[2].serial_dev, ( uint8_t* ) dev_ctx[2].buf, sizeof(dev_ctx[2].buf), 0);

        dev_ctx[2].write_buf_idx = 0;
        dev_ctx[2].read_buf_idx = 1;

        counter++;
    }

    k_thread_create(&rx_thread_data, bm_rx_stack,
            K_THREAD_STACK_SIZEOF(bm_rx_stack),
            (k_thread_entry_t)bm_serial_rx_thread,
            (void *)dev, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);
    
    k_thread_create(&tx_thread_data, bm_tx_stack,
            K_THREAD_STACK_SIZEOF(bm_tx_stack),
            (k_thread_entry_t)bm_serial_tx_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

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

static struct net_pkt *bm_rx(const struct device *dev)
{
    struct bm_stm32_hal_dev_data *dev_data;
    struct net_pkt *pkt;
    size_t total_len;

    dev_data = dev->data;
    __ASSERT_NO_MSG(dev_data != NULL);

    // TODO: Move logic that is currently embedded in rx_thread to here
    // TODO: Add interface fragment to packet

    return NULL;
}

static int bm_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct bm_stm32_hal_dev_data *dev_data = dev->data;
	int res = 0;
	size_t total_len;

    (void)dev_data;
    (void)total_len;

	__ASSERT_NO_MSG(pkt != NULL);
	__ASSERT_NO_MSG(pkt->frags != NULL);
	__ASSERT_NO_MSG(dev != NULL);
	__ASSERT_NO_MSG(dev_data != NULL);

    k_mutex_lock(&dev_data->tx_mutex, K_FOREVER);


    // Get total length of packet
    total_len = net_pkt_get_len(pkt);
	if (total_len > NET_BM_MAX_FRAME_SIZE) {
		LOG_ERR("PKT too big");
		res = -EIO;
		goto error;
	}


    // Read packet into tx buffer
	if (net_pkt_read(pkt, dev_data->tx_frame_buf, total_len)) {
        LOG_ERR("Couldn't read packet");
		res = -ENOBUFS;
		goto error;
	}

    // Check for free space in TX Queue
    if( k_msgq_num_free_get( &tx_queue ) == 0 )
    {
        LOG_ERR( "TX FIFO is full" );
        res = -ENOSPC;
        goto error;
    }

    // Calculate CRC32_IEEE
    uint32_t crc32 = crc32_ieee( dev_data->tx_frame_buf, total_len );

    /* Add frame to TX Contiguous Mem */
    memcpy( (uint8_t *)&tx_payload_buf[tx_payload_idx * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE], dev_data->tx_frame_buf, total_len);
    /* Add CRC32 after Frame */
    memcpy( (uint8_t *)&tx_payload_buf[(tx_payload_idx * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE) + total_len], ( uint8_t* )&crc32, sizeof(crc32));

    /* Add msg to TX Message Queue (for TX Task to consume */ 
    bm_msg_t tx_msg = { .frame_addr = &tx_payload_buf[tx_payload_idx * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE], .frame_length = ( total_len + sizeof(crc32) ) };
    if( k_msgq_put(&tx_queue, &tx_msg, K_NO_WAIT) )
    {
        LOG_ERR( "Failed to queue message in TX FIFO" );
        res = -ENOMSG;
        goto error;
    }

    /* Update index for storing next TX Payload */
    tx_payload_idx++;
    if (tx_payload_idx >= CONFIG_BM_STM32_HAL_NUM_FRAMES)
    {
        tx_payload_idx = 0;
    }

    res = 0;
error:
	k_mutex_unlock(&dev_data->tx_mutex);
    
    return res;
}

static const struct bristlemouth_api bm_api = {
	.iface_api.init = bm_iface_init,
	.get_capabilities = bm_stm32_hal_get_capabilities,
	.set_config = bm_stm32_hal_set_config,
	.send = bm_tx,
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

#define BM_STM32_HAL_MTU NET_BM_MTU
#define BM_STM32_HAL_FRAME_SIZE_MAX (BM_STM32_HAL_MTU + 18)

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
// Add support for introspecting MAC header to see if message needs forwarding
// Add support for tagging packets in RX thread with the interface they came from
// Add support for routing packets TX to only the interfaces they need to go to
// Add support for forwarding multicast based on the destination addr (ff02::1 vs ff03::1)
// Investigate CRC failure - potentially missing proper mutexing for RX pipe

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


// Helpers:

// Iterate/Read frags:
	// size_t bytes = 0;
    // struct net_buf *buf = pkt->frags;
    // int i = 0;
	// while (buf) {

    //     uint8_t* d = buf->data;
    //     LOG_INF("Frag %d: size=%d", i, buf->len);
    //     LOG_INF("First values of frag: %d %d %d %d %d %d %d %d %d %d %d %d %d %d", 
    //         d[0], d[1], d[2], d[3], d[4], d[5],
    //         d[6], d[7], d[8], d[9], d[10], d[11],
    //         d[12], d[13]);

	// 	bytes += buf->len;
	// 	buf = buf->frags;
    //     i++;
	// }

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


// net_pkt.c
// K_MEM_SLAB_DEFINE(rx_pkts, sizeof(struct net_pkt), CONFIG_NET_PKT_RX_COUNT, 4);
// K_MEM_SLAB_DEFINE(tx_pkts, sizeof(struct net_pkt), CONFIG_NET_PKT_TX_COUNT, 4);