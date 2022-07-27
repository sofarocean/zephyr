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

#include <drivers/uart.h>
#include <sys/printk.h>
#include <sys/crc.h>
#include <drivers/gpio.h>

#include "bm_stm32_hal_priv.h"
#include "manchester.h"
#include "murmur_hash.h"

static volatile bm_ctx_t dev_ctx[CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT];

static struct k_thread tx_thread_data;
static struct k_thread rx_thread_data;

/* Buffer for received frame payloads */
static volatile uint8_t rx_payload_buf[CONFIG_BM_STM32_HAL_NUM_FRAMES * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE];
static uint8_t rx_payload_idx = 0;

/* Buffer for transmitted frame payloads */
static uint8_t tx_payload_buf[CONFIG_BM_STM32_HAL_NUM_FRAMES * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE];
static uint8_t tx_payload_idx = 0;

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
static void bm_serial_rx_thread(void)
{
    uint16_t computed_crc16;
    uint16_t received_crc16;
    int retval;
    int i;

    /* Buffer to store Manchester Decoded RX data */
    uint8_t man_decode_buf[ CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE ] = {0};
    uint16_t man_decode_len = 0;
    int8_t decoded_nibble = 0;
    uint8_t preamble_err = 0;

    /* Local buf to store DMA data */
    bm_rx_t incoming_rx_data;

    /* Dev from message queue */
    uint8_t dev_idx;

    LOG_DBG("BM Serial RX thread started");

    while (1)
    {
        k_msgq_get(&encoded_rx_queue, &dev_idx, K_FOREVER);

        /* Copy over DMA data and then release semaphore for DMA cb */
        k_sem_take(&dma_idx_sem , K_FOREVER);
        incoming_rx_data.length = dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].read_buf_idx].length;
        memcpy(incoming_rx_data.buf, (const uint8_t * ) dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].read_buf_idx].buf, incoming_rx_data.length);
        k_sem_give(&dma_idx_sem);

        /* Check if length is even */
        if (incoming_rx_data.length & 0x01)
        {
            LOG_ERR("Packet size is odd");
            continue;
        }

        /* Check Preamble */ 
        preamble_err = 0;
        for (i=0; i < CONFIG_BM_STM32_HAL_PREAMBLE_LEN; i++)
        {
            if ( incoming_rx_data.buf[i] != CONFIG_BM_STM32_HAL_PREAMBLE_VAL )
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

        /* Remove preamble bytes from packet length */
        incoming_rx_data.length -= CONFIG_BM_STM32_HAL_PREAMBLE_LEN;

        memset(man_decode_buf, 0, sizeof(man_decode_buf));

        /* Decode manchester - Skip Preamble bytes */
        man_decode_len = incoming_rx_data.length/2;
        for ( i = (CONFIG_BM_STM32_HAL_PREAMBLE_LEN/2); i < man_decode_len + (CONFIG_BM_STM32_HAL_PREAMBLE_LEN/2); i++ )
        {
            decoded_nibble = BM_MAN_DECODE_TABLE[incoming_rx_data.buf[(2*i)]];
            if (decoded_nibble >= 0)
            {
                man_decode_buf[i - (CONFIG_BM_STM32_HAL_PREAMBLE_LEN/2)] |= (((uint8_t) decoded_nibble) & 0xF);
            }
            else
            {
                LOG_ERR("Invalid Manchester value");
            }

            decoded_nibble = BM_MAN_DECODE_TABLE[incoming_rx_data.buf[(2*i)+1]];
            if (decoded_nibble >= 0)
            {
                man_decode_buf[i - (CONFIG_BM_STM32_HAL_PREAMBLE_LEN/2)] |= ((((uint8_t) decoded_nibble) & 0xF) << 4);
            }
            else
            {
                LOG_ERR("Invalid Manchester value");
            }
        }

        if (man_decode_len == 0)
        {
            LOG_ERR("Received Frame Length of 0. Skipping");
            continue;
        }

        /* Verify CRC16 (Bristlemouth Packet = header + payload)*/
        computed_crc16 = crc16_ccitt(0, man_decode_buf, man_decode_len - sizeof(bm_crc_t));
        received_crc16 = man_decode_buf[man_decode_len - 2];
        received_crc16 |= (man_decode_buf[man_decode_len - 1] << 8);

        if (computed_crc16 != received_crc16)
        {
            LOG_ERR("CRC16 received: %d vs. computed: %d, discarding\n", received_crc16, computed_crc16);
            continue;
        }

        /* Update frame length with CRC16 removal */
        man_decode_len -= sizeof(bm_crc_t);

        if(k_msgq_num_free_get(&rx_queue))
        {
            /* Add frame to RX Contiguous Mem */
            linear_memcpy(( uint8_t * ) &rx_payload_buf[rx_payload_idx * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE], man_decode_buf, man_decode_len);

            /* Add msg to RX Message Queue (for ieee802154_bm_serial.c RX Task to consume */ 
            bm_msg_t rx_msg = { .frame_addr = &rx_payload_buf[rx_payload_idx * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE], .frame_length = man_decode_len};
            retval = k_msgq_put(&rx_queue, &rx_msg, K_NO_WAIT);
            if (retval)
            {
                LOG_ERR("Message could not be added to Queue");
            }

            /* Update index for storing next RX Payload */
            rx_payload_idx++;
            if (rx_payload_idx >= CONFIG_BM_STM32_HAL_NUM_FRAMES)
            {
                rx_payload_idx = 0;
            }
        }
        else
        {
            LOG_ERR("RX MessageQueue full, dropping message. Get faster!");
        }
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
    LOG_DBG("BM Serial TX thread started");

    bm_msg_t msg;
    volatile uint8_t* frame_addr;
    uint16_t frame_length;
    uint16_t i;
    uint8_t n;
    uint16_t encoded_val;

    /* Create a buf to pass to DMA TX (account for preamble)*/
    uint8_t tx_enc_buf[2*(CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE) + CONFIG_BM_STM32_HAL_PREAMBLE_LEN];
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

        /* Send out preamble*/
        for ( tx_buf_ctr = 0; tx_buf_ctr < CONFIG_BM_STM32_HAL_PREAMBLE_LEN; tx_buf_ctr++)
        {
            tx_enc_buf[tx_buf_ctr] = CONFIG_BM_STM32_HAL_PREAMBLE_VAL;
        }
        
        /* Stuff payload one byte at a time */
        for ( i=0; i < frame_length; i++)
        {
            encoded_val = BM_MAN_ENCODE_TABLE[frame_addr[i]];
            tx_enc_buf[tx_buf_ctr++] = (uint8_t) (encoded_val & 0xFF);
            tx_enc_buf[tx_buf_ctr++] = (uint8_t) ((encoded_val >> 8) & 0xFF);
        }

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
                uart_tx(dev_ctx[n].serial_dev, tx_enc_buf, tx_buf_ctr, 10 * USEC_PER_MSEC);
            }
        }
    }
}

struct k_msgq* bm_serial_get_rx_msgq_handler(void)
{
    /* Simple getter for allowing ieee802154_bm_serial.c to listen to message queue */
    return &rx_queue;
}

/* Computes Bristlemouth Packet CRC16, stores in Tx Frame Buffer, and adds message to Queue for Tx Task  */
int bm_serial_frm_put(bm_frame_t* bm_frm)
{
    int retval = -1;
    uint16_t frame_length = bm_frm->frm_hdr.payload_length + sizeof(bm_frame_header_t);

    if (sizeof(tx_payload_buf) < frame_length + sizeof(bm_crc_t))
    {
        LOG_ERR("Frame is too large. Ignoring");
        goto out;
    }

    /* Computed on the entire packet, using CCITT */
    uint16_t computed_crc16 = crc16_ccitt(0, (uint8_t *) bm_frm, frame_length);

    if (k_msgq_num_free_get(&tx_queue))
    {
        /* Add frame to TX Contiguous Mem */
        linear_memcpy((uint8_t *) &tx_payload_buf[tx_payload_idx * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE], ( uint8_t* ) bm_frm, frame_length);
        /* Add CRC16 after Frame */
        linear_memcpy((uint8_t *) &tx_payload_buf[(tx_payload_idx * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE) + frame_length], ( uint8_t* ) &computed_crc16, sizeof(bm_crc_t));

        /* Update the frame length with CRC16 */
        frame_length += sizeof(bm_crc_t);

        /* Add msg to TX Message Queue (for TX Task to consume */ 
        bm_msg_t tx_msg = { .frame_addr = &tx_payload_buf[tx_payload_idx * CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE], .frame_length = frame_length};
        retval = k_msgq_put(&tx_queue, &tx_msg, K_FOREVER);

        /* Update index for storing next TX Payload */
        tx_payload_idx++;
        if (tx_payload_idx >= CONFIG_BM_STM32_HAL_NUM_FRAMES)
        {
            tx_payload_idx = 0;
        }
    }
    else
    {
        LOG_ERR("TX MessageQueue full, dropping message!");
    }
out:
    return retval;	
}

static int bm_initialize(const struct device *dev)
{
    int ret = 0;
    struct bm_stm32_hal_dev_data *dev_data;

    __ASSERT_NO_MSG(dev != NULL);

	dev_data = dev->data;

    __ASSERT_NO_MSG(dev_data != NULL);

    // TODO: Setup MAC from uuid using murmur_32 hash or random

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
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);
    
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
    (void)dev_data;

    // TODO: Take bm_frame from rx queue and put it into a packet
    // Looks something like:

    // pkt = net_pkt_rx_alloc_with_buffer(get_iface(dev_data, *vlan_tag),
	// 				   total_len, AF_UNSPEC, 0, K_MSEC(100));
	// if (!pkt) {
	// 	LOG_ERR("Failed to obtain RX buffer");
	// 	goto release_desc;
	// }

	// if (net_pkt_write(pkt, dma_buffer, total_len)) {
	// 	LOG_ERR("Failed to append RX buffer to context buffer");
	// 	net_pkt_unref(pkt);
	// 	pkt = NULL;
	// 	goto release_desc;
	// }

    return NULL;
}

// TODO: In RX thread, call the above with something like
// while ((pkt = bm_rx(dev)) != NULL) {
//     res = net_recv_data(net_pkt_iface(pkt), pkt);
//     if (res < 0) {
//         LOG_ERR("Failed to enqueue frame into RX queue: %d", res);
//         net_pkt_unref(pkt);
//     }
// }

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

    // TODO: Put into tx queue as a bm_frame
    
    return res;
}

static const struct bristlemouth_api bm_api = {
	.iface_api.init = bm_iface_init,
	.get_capabilities = bm_stm32_hal_get_capabilities,
	.set_config = bm_stm32_hal_set_config,
	.send = bm_tx,
};

#define ST_OUI_B0 0x00
#define ST_OUI_B1 0x80
#define ST_OUI_B2 0xE1
static struct bm_stm32_hal_dev_data bm0_data = {
	.mac_addr = {
		ST_OUI_B0,
		ST_OUI_B1,
		ST_OUI_B2,
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

// TODO:
// Set up initial link-local address (or does zephyr do this automatically in ipv6 layer?)
// IPv6 ND exists in zephyr IP layer. Need to validate functionality
// Set up well known multicast addresses
// Add support for introspecting MAC header to see if message needs forwarding
// Pare down header info from bm_serial