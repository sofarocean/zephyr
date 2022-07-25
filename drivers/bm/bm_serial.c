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

#include <logging/log.h>

#include <init.h>
#include <stdbool.h>
#include <drivers/uart.h>
#include <sys/printk.h>
#include <sys/crc.h>
#include <drivers/gpio.h>

#include <drivers/bm/bm_serial.h>

#ifdef CONFIG_BM_DFU
#include <drivers/bm/bm_dfu.h>
#endif

LOG_MODULE_REGISTER(bm_serial, CONFIG_BM_LOG_LEVEL);

static volatile bm_ctx_t dev_ctx[CONFIG_BM_MAX_SERIAL_DEV_COUNT];

static struct k_thread tx_thread_data;
static struct k_thread rx_thread_data;

/* Buffer for received frame payloads */
static volatile uint8_t rx_payload_buf[CONFIG_BM_NUM_FRAMES * CONFIG_BM_MAX_FRAME_SIZE];
static uint8_t rx_payload_idx = 0;

#ifdef CONFIG_BM_DFU
/* Buffer for received DFU frame payloads */
static uint8_t dfu_rx_payload_buf[CONFIG_BM_DFU_NUM_FRAMES * CONFIG_BM_MAX_FRAME_SIZE];
static uint8_t dfu_rx_payload_idx = 0;

/* DFU message queue */
static struct k_msgq* _dfu_rx_queue = NULL;
#endif

/* Buffer for transmitted frame payloads */
static uint8_t tx_payload_buf[CONFIG_BM_NUM_FRAMES * CONFIG_BM_MAX_FRAME_SIZE];
static uint8_t tx_payload_idx = 0;

/* RX/TX Threads and associated message Queues */
K_THREAD_STACK_DEFINE(bm_tx_stack, CONFIG_BM_TASK_STACK_SIZE);
K_MSGQ_DEFINE(tx_queue, sizeof(bm_msg_t), CONFIG_BM_NUM_FRAMES, 4);
K_THREAD_STACK_DEFINE(bm_rx_stack, CONFIG_BM_TASK_STACK_SIZE);
K_MSGQ_DEFINE(rx_queue, sizeof(bm_msg_t), CONFIG_BM_NUM_FRAMES, 4);

K_MSGQ_DEFINE(encoded_rx_queue, sizeof(uint8_t), CONFIG_BM_NUM_FRAMES, 4);

/* Semaphore for ISR and RX_Task to signal availability of Decoded Frame */
K_SEM_DEFINE(dma_idx_sem, 1, 1);

const uint16_t MAN_ENCODE_TABLE[256] = 
{
    0xAAAA, 0xAAA9, 0xAAA6, 0xAAA5, 0xAA9A, 0xAA99, 0xAA96, 0xAA95, 0xAA6A, 0xAA69, 0xAA66, 0xAA65, 0xAA5A, 0xAA59, 0xAA56, 0xAA55,
    0xA9AA, 0xA9A9, 0xA9A6, 0xA9A5, 0xA99A, 0xA999, 0xA996, 0xA995, 0xA96A, 0xA969, 0xA966, 0xA965, 0xA95A, 0xA959, 0xA956, 0xA955,
    0xA6AA, 0xA6A9, 0xA6A6, 0xA6A5, 0xA69A, 0xA699, 0xA696, 0xA695, 0xA66A, 0xA669, 0xA666, 0xA665, 0xA65A, 0xA659, 0xA656, 0xA655,
    0xA5AA, 0xA5A9, 0xA5A6, 0xA5A5, 0xA59A, 0xA599, 0xA596, 0xA595, 0xA56A, 0xA569, 0xA566, 0xA565, 0xA55A, 0xA559, 0xA556, 0xA555,
    0x9AAA, 0x9AA9, 0x9AA6, 0x9AA5, 0x9A9A, 0x9A99, 0x9A96, 0x9A95, 0x9A6A, 0x9A69, 0x9A66, 0x9A65, 0x9A5A, 0x9A59, 0x9A56, 0x9A55,
    0x99AA, 0x99A9, 0x99A6, 0x99A5, 0x999A, 0x9999, 0x9996, 0x9995, 0x996A, 0x9969, 0x9966, 0x9965, 0x995A, 0x9959, 0x9956, 0x9955,
    0x96AA, 0x96A9, 0x96A6, 0x96A5, 0x969A, 0x9699, 0x9696, 0x9695, 0x966A, 0x9669, 0x9666, 0x9665, 0x965A, 0x9659, 0x9656, 0x9655,
    0x95AA, 0x95A9, 0x95A6, 0x95A5, 0x959A, 0x9599, 0x9596, 0x9595, 0x956A, 0x9569, 0x9566, 0x9565, 0x955A, 0x9559, 0x9556, 0x9555,
    0x6AAA, 0x6AA9, 0x6AA6, 0x6AA5, 0x6A9A, 0x6A99, 0x6A96, 0x6A95, 0x6A6A, 0x6A69, 0x6A66, 0x6A65, 0x6A5A, 0x6A59, 0x6A56, 0x6A55,
    0x69AA, 0x69A9, 0x69A6, 0x69A5, 0x699A, 0x6999, 0x6996, 0x6995, 0x696A, 0x6969, 0x6966, 0x6965, 0x695A, 0x6959, 0x6956, 0x6955,
    0x66AA, 0x66A9, 0x66A6, 0x66A5, 0x669A, 0x6699, 0x6696, 0x6695, 0x666A, 0x6669, 0x6666, 0x6665, 0x665A, 0x6659, 0x6656, 0x6655,
    0x65AA, 0x65A9, 0x65A6, 0x65A5, 0x659A, 0x6599, 0x6596, 0x6595, 0x656A, 0x6569, 0x6566, 0x6565, 0x655A, 0x6559, 0x6556, 0x6555,
    0x5AAA, 0x5AA9, 0x5AA6, 0x5AA5, 0x5A9A, 0x5A99, 0x5A96, 0x5A95, 0x5A6A, 0x5A69, 0x5A66, 0x5A65, 0x5A5A, 0x5A59, 0x5A56, 0x5A55,
    0x59AA, 0x59A9, 0x59A6, 0x59A5, 0x599A, 0x5999, 0x5996, 0x5995, 0x596A, 0x5969, 0x5966, 0x5965, 0x595A, 0x5959, 0x5956, 0x5955,
    0x56AA, 0x56A9, 0x56A6, 0x56A5, 0x569A, 0x5699, 0x5696, 0x5695, 0x566A, 0x5669, 0x5666, 0x5665, 0x565A, 0x5659, 0x5656, 0x5655,
    0x55AA, 0x55A9, 0x55A6, 0x55A5, 0x559A, 0x5599, 0x5596, 0x5595, 0x556A, 0x5569, 0x5566, 0x5565, 0x555A, 0x5559, 0x5556, 0x5555,
};

const int8_t MAN_DECODE_TABLE[256] = 
{
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, 0xF, 0xE, -1, -1, 0xD, 0xC, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, 0xB, 0xA, -1, -1, 0x9, 0x8, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, 0x7, 0x6, -1, -1, 0x5, 0x4, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, 0x3, 0x2, -1, -1, 0x1, 0x0, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

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
    uint8_t man_decode_buf[ CONFIG_BM_MAX_FRAME_SIZE ] = {0};
    uint16_t man_decode_len = 0;
    int8_t decoded_nibble = 0;
    uint8_t preamble_err = 0;

    /* Local buf to store DMA data */
    bm_rx_t incoming_rx_data;

    /* Dev from message queue */
    uint8_t dev_idx;
    uint8_t payload_type = 0;

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
        for (i=0; i < CONFIG_BM_PREAMBLE_LEN; i++)
        {
            if ( incoming_rx_data.buf[i] != CONFIG_BM_PREAMBLE_VAL )
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
        incoming_rx_data.length -= CONFIG_BM_PREAMBLE_LEN;

        memset(man_decode_buf, 0, sizeof(man_decode_buf));

        /* Decode manchester - Skip Preamble bytes */
        man_decode_len = incoming_rx_data.length/2;
        for ( i = (CONFIG_BM_PREAMBLE_LEN/2); i < man_decode_len + (CONFIG_BM_PREAMBLE_LEN/2); i++ )
        {
            decoded_nibble = MAN_DECODE_TABLE[incoming_rx_data.buf[(2*i)]];
            if (decoded_nibble >= 0)
            {
                man_decode_buf[i - (CONFIG_BM_PREAMBLE_LEN/2)] |= (((uint8_t) decoded_nibble) & 0xF);
            }
            else
            {
                LOG_ERR("Invalid Manchester value");
            }

            decoded_nibble = MAN_DECODE_TABLE[incoming_rx_data.buf[(2*i)+1]];
            if (decoded_nibble >= 0)
            {
                man_decode_buf[i - (CONFIG_BM_PREAMBLE_LEN/2)] |= ((((uint8_t) decoded_nibble) & 0xF) << 4);
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

        payload_type = man_decode_buf[1];
        
        switch (payload_type)
        {
            case BM_IEEE802154:
                if (k_msgq_num_free_get(&rx_queue))
                {
                    /* Add frame to RX Contiguous Mem */
                    memcpy(&rx_payload_buf[rx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE], man_decode_buf, man_decode_len);

                    /* Add msg to RX Message Queue (for ieee802154_bm_serial.c RX Task to consume */ 
                    bm_msg_t rx_msg = { .frame_addr = &rx_payload_buf[rx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE], .frame_length = man_decode_len};
                    retval = k_msgq_put(&rx_queue, &rx_msg, K_NO_WAIT);
                    if (retval)
                    {
                        LOG_ERR("Message could not be added to Queue");
                    }

                    /* Update index for storing next RX Payload */
                    rx_payload_idx++;
                    if (rx_payload_idx >= CONFIG_BM_NUM_FRAMES)
                    {
                        rx_payload_idx = 0;
                    }
                }
                else
                {
                    LOG_ERR("RX MessageQueue full, dropping message. Get faster!");
                }
                break;
            #ifdef CONFIG_BM_DFU
            case BM_DFU:
                if (k_msgq_num_free_get(_dfu_rx_queue))
                {
                    /* Add frame to RX Contiguous Mem */
                    memcpy( &dfu_rx_payload_buf[dfu_rx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE], man_decode_buf, man_decode_len);

                    bm_msg_t rx_msg = { .frame_addr = &dfu_rx_payload_buf[dfu_rx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE], .frame_length = man_decode_len};
                    retval = k_msgq_put(_dfu_rx_queue, &rx_msg, K_NO_WAIT);
                    if (retval)
                    {
                        LOG_ERR("Message could not be added to Queue");
                        // NOTE: Should we exit early here and leave the index where its at since the current one is unused?
                        // Since this is the only place dfu_rx_payload_idx is used, and it was given to _dfu_rx_queue as a buffer pointer
                        // we need to protect the lifetime of the payload until it has been removed from the queue.
                    }

                    /* Update index for storing next DFU RX Payload */
                    dfu_rx_payload_idx++;
                    if (dfu_rx_payload_idx >= CONFIG_BM_DFU_NUM_FRAMES)
                    {
                        dfu_rx_payload_idx = 0;
                    }
                }
                else
                {
                    LOG_ERR("DFU RX MessageQueue full, dropping message. Get faster!");
                }
                break;
            #endif
            case BM_GENERIC:
            default:
                break;
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
            for (i = 0; i < CONFIG_BM_MAX_SERIAL_DEV_COUNT; i++)
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
            for (i = 0; i < CONFIG_BM_MAX_SERIAL_DEV_COUNT; i++)
            {
                if ( dev == dev_ctx[i].serial_dev)
                {
                    dev_idx = i;
                    buf_len = sizeof(dev_ctx[i].buf);
                }
            }

            if (pos != old_pos) 
            {
                /* Current position is over previous one */
                if (pos > old_pos) 
                {
                    /* We are in "linear" mode
                          Process data directly by subtracting "pointers" */
                    memcpy(( uint8_t* )dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].write_buf_idx].buf, &buf[old_pos], pos-old_pos);
                    dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].write_buf_idx].length = (pos - old_pos);
                }
                else
                {
                    
                    /* We are in "overflow" mode
                       First process data to the end of buffer */

                    /* Check and continue with beginning of buffer */
                    memcpy(( uint8_t* ) dev_ctx[dev_idx].encoded_rx_buf[dev_ctx[dev_idx].write_buf_idx].buf, &buf[old_pos], buf_len-old_pos);
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
    uint8_t tx_enc_buf[2*(CONFIG_BM_MAX_FRAME_SIZE) + CONFIG_BM_PREAMBLE_LEN];
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
        for ( tx_buf_ctr = 0; tx_buf_ctr < CONFIG_BM_PREAMBLE_LEN; tx_buf_ctr++)
        {
            tx_enc_buf[tx_buf_ctr] = CONFIG_BM_PREAMBLE_VAL;
        }
        
        /* Stuff payload one byte at a time */
        for ( i=0; i < frame_length; i++)
        {
            encoded_val = MAN_ENCODE_TABLE[frame_addr[i]];
            tx_enc_buf[tx_buf_ctr++] = (uint8_t) (encoded_val & 0xFF);
            tx_enc_buf[tx_buf_ctr++] = (uint8_t) ((encoded_val >> 8) & 0xFF);
        }

        for ( n=0; n < CONFIG_BM_MAX_SERIAL_DEV_COUNT; n++)
        {
            if (dev_ctx[n].serial_dev != NULL)
            {
                /* Try grabbing semaphore  (wait forever if not available) */
                k_sem_take(( struct k_sem* ) &dev_ctx[n].sem, K_FOREVER);
            }
        }

        for ( n=0; n < CONFIG_BM_MAX_SERIAL_DEV_COUNT; n++)
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
        memcpy(&tx_payload_buf[tx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE], bm_frm, frame_length);
        /* Add CRC16 after Frame */
        memcpy(&tx_payload_buf[(tx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE) + frame_length], &computed_crc16, sizeof(bm_crc_t));

        /* Update the frame length with CRC16 */
        frame_length += sizeof(bm_crc_t);

        /* Add msg to TX Message Queue (for TX Task to consume */ 
        bm_msg_t tx_msg = { .frame_addr = &tx_payload_buf[tx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE], .frame_length = frame_length};
        retval = k_msgq_put(&tx_queue, &tx_msg, K_FOREVER);

        /* Update index for storing next TX Payload */
        tx_payload_idx++;
        if (tx_payload_idx >= CONFIG_BM_NUM_FRAMES)
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

int bm_serial_init( const struct device *arg )
{
    ARG_UNUSED(arg);

    static const struct device* _dev;
    uint8_t counter = 0;
    uint32_t interframe_delay_us;
    uint32_t* baud_rate_addr;
    k_tid_t thread_id;

    _dev = device_get_binding(CONFIG_BM_SERIAL_DEV_NAME_0);
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

    _dev = device_get_binding(CONFIG_BM_SERIAL_DEV_NAME_1);
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

    _dev = device_get_binding(CONFIG_BM_SERIAL_DEV_NAME_2);
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

    thread_id = k_thread_create(&rx_thread_data, bm_rx_stack,
            K_THREAD_STACK_SIZEOF(bm_rx_stack),
            (k_thread_entry_t)bm_serial_rx_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

    if (thread_id == NULL)
    {
        LOG_ERR("BM Serial RX thread not created?");
        return -1;
    }
    
    thread_id = k_thread_create(&tx_thread_data, bm_tx_stack,
            K_THREAD_STACK_SIZEOF(bm_tx_stack),
            (k_thread_entry_t)bm_serial_tx_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

    if (thread_id == NULL)
    {
        LOG_ERR("BM Serial TX thread not created?");
        return -1;
    }

    #ifdef CONFIG_BM_DFU
    _dfu_rx_queue = bm_dfu_get_transport_service_queue();
    #endif

    return 0;
}

SYS_INIT( bm_serial_init, POST_KERNEL, 0 );
