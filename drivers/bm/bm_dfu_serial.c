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
#include <cobs.h>
#include <sys/crc.h>
#include <drivers/gpio.h>

#include <zephyr/bm/bm_dfu.h>
#include <zephyr/bm/bm_dfu_serial.h>

LOG_MODULE_REGISTER(bm_dfu_serial, CONFIG_BM_LOG_LEVEL);

/* RX/TX Threads and associated message Queues */
K_THREAD_STACK_DEFINE(bm_dfu_tx_stack, CONFIG_BM_DFU_TASK_STACK_SIZE);
K_MSGQ_DEFINE(dfu_tx_queue, sizeof(bm_msg_t), CONFIG_BM_DFU_NUM_FRAMES, 4);
K_THREAD_STACK_DEFINE(bm_dfu_rx_stack, CONFIG_BM_DFU_TASK_STACK_SIZE);
K_MSGQ_DEFINE(dfu_rx_queue, sizeof(bm_msg_t), CONFIG_BM_DFU_NUM_FRAMES, 4);

static struct k_thread dfu_tx_thread_data;
static struct k_thread dfu_rx_thread_data;

/* Buffer to store encoded frame before TX */
uint8_t cobs_encoding_buffer[COBS_ENCODE_DST_BUF_LEN_MAX(CONFIG_BM_MAX_FRAME_SIZE) + 1];

/* Semaphore for ISR and RX_Task to signal availability of Decoded Frame */
K_SEM_DEFINE(cobs_decode_sem, 0, 1);

/* Semaphore for ISR and RX_Task to signal processing of frame has finished 
   Begin with initial value of 1 so that ISR can initiate RX*/
K_SEM_DEFINE(processing_sem, 1, 1);

/* Semaphore for notifying DFU task that new frame is available */
K_SEM_DEFINE(dfu_sem, 1, 1);

static dfu_serial_ctx_t dfu_serial_ctx;

/* COBS Decoding */
static void bm_parse_and_store(uint8_t *rx_byte)
{
    uint8_t byte_in = *rx_byte;	

    static volatile uint16_t len = 0;
    static volatile uint8_t await_alignment = 0;
    static uint8_t input_buffer[ CONFIG_BM_MAX_FRAME_SIZE ];

    if (await_alignment)
    {
        if ( byte_in == COBS_DELIMITER )
        {
            await_alignment = 0;
            return;
        }
    }
    else
    {
        if( byte_in == COBS_DELIMITER )
        {
            if (len != 0)
            {
                cobs_decode_result_t ret = cobs_decode( dfu_serial_ctx.cobs_decoding_buf[dfu_serial_ctx.write_buf_idx].buf, CONFIG_BM_MAX_FRAME_SIZE, input_buffer, len );
                if( ret.status == COBS_DECODE_OK)
                {
                    dfu_serial_ctx.cobs_decoding_buf[dfu_serial_ctx.write_buf_idx].length = ret.out_len;

                    /* First check if the RX_Task has finished processing read_buf */
                    if ( k_sem_take(&processing_sem, K_NO_WAIT))
                    {
                        LOG_ERR("RX Task has not finished processing data. Overwriting current index of A/B Buffer.");
                    }
                    else
                    {
                        /* Flip idx of read and write buffers */
                        dfu_serial_ctx.read_buf_idx = dfu_serial_ctx.write_buf_idx;
                        dfu_serial_ctx.write_buf_idx = 1 - dfu_serial_ctx.write_buf_idx;
                    }

                    /* Notify RX Task that frame is ready to be processed */
                    k_sem_give(&cobs_decode_sem);
                }
                else
                {
                    LOG_ERR( "Failed to decode COBS frame" );
                }
            }
            else
            {
                LOG_ERR("Received an errant 0x00 byte");
            }
            len = 0;
        }
        else
        {
            input_buffer[len] = byte_in;
            len++;
        }
    }

    return;
}

static void bm_dfu_serial_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t byte;

    uart_irq_update(dev);

    if (uart_irq_is_pending(dev) && uart_irq_rx_ready(dev)) 
    {
        if ( uart_fifo_read(dev, &byte, 1) )
        {
            bm_parse_and_store(&byte);
        }
    }
}

/**
 * BM Serial TX Thread
 */
static void bm_dfu_serial_tx_thread(void)
{
    bm_msg_t msg;
    uint8_t* frame_addr;
    uint16_t frame_length;
    cobs_encode_result_t enc_retv;
    uint16_t i;

    while (1)
    {
        k_msgq_get(&dfu_tx_queue, &msg, K_FOREVER);

        frame_addr = msg.frame_addr;
        frame_length = msg.frame_length;

        if (frame_length > sizeof(cobs_encoding_buffer))
        {
            LOG_ERR("TX Msg size too large to send. Ignoring");
            continue;
        }

        enc_retv = cobs_encode(cobs_encoding_buffer, CONFIG_BM_MAX_FRAME_SIZE, frame_addr, frame_length);
        /* COBS Library does not delimit with 0x00. We can stuff the next value in the array with it manually. 
           The size of the array should be 1 larger than the max possible encoded length, so this check should
           theoretically not fail */
        if (enc_retv.out_len == sizeof(cobs_encoding_buffer))
        {
            LOG_ERR("The COBS encoded buffer length has exceeded the allotable size");
            continue;
        }

		if (enc_retv.status == COBS_ENCODE_OK)
		{
            cobs_encoding_buffer[enc_retv.out_len] = COBS_DELIMITER;

            /* Send out UART one byte at a time */
            i = 0;
            while (i < enc_retv.out_len + 1)
            {
                uart_poll_out(dfu_serial_ctx.serial_dev, cobs_encoding_buffer[i++]);
            }
		}
    }
}

/**
 * BM Serial RX Thread
 */
static void bm_dfu_serial_rx_thread(void)
{
    uint16_t computed_crc16;
    uint16_t received_crc16;
    uint16_t frame_length;
    int retval;
    uint8_t payload_type = 0;

    while (1)
    {
        /* Wait on Producer to finish writing out decoded frame */
        k_sem_take(&cobs_decode_sem, K_FOREVER);

        frame_length = dfu_serial_ctx.cobs_decoding_buf[dfu_serial_ctx.read_buf_idx].length;

        if (frame_length == 0)
        {
            LOG_ERR("Received Frame Length of 0. Skipping");
            k_sem_give(&processing_sem);
            continue;
        }

        /* Verify CRC16 */
        computed_crc16 = crc16_ccitt(0, dfu_serial_ctx.cobs_decoding_buf[dfu_serial_ctx.read_buf_idx].buf, frame_length - sizeof(bristlemouth_crc_t));
        received_crc16 = dfu_serial_ctx.cobs_decoding_buf[dfu_serial_ctx.read_buf_idx].buf[frame_length - 2];
        received_crc16 |= (dfu_serial_ctx.cobs_decoding_buf[dfu_serial_ctx.read_buf_idx].buf[frame_length - 1] << 8);

        if (computed_crc16 != received_crc16)
        {
            LOG_ERR("CRC16 received: %d vs. computed: %d, discarding\n", received_crc16, computed_crc16);
            k_sem_give(&processing_sem);
            continue;
        }

        /* Update frame length with CRC16 removal */
        frame_length -= sizeof(bristlemouth_crc_t);
        payload_type = dfu_serial_ctx.cobs_decoding_buf[dfu_serial_ctx.read_buf_idx].buf[1];
        
        switch (payload_type)
        {
            case BRISTLEMOUTH_DFU:
                if (k_sem_take(dfu_serial_ctx.dfu_sem , K_NO_WAIT) != 0)
                {
                    LOG_ERR("Can't take DFU semaphore");
                    break;
                }
                if (k_msgq_num_free_get(dfu_serial_ctx.dfu_rx_queue))
                {
                    /* Add frame to RX Contiguous Mem */
                    memcpy( &dfu_serial_ctx.rx_payload_buf[dfu_serial_ctx.rx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE], dfu_serial_ctx.cobs_decoding_buf[dfu_serial_ctx.read_buf_idx].buf, frame_length);
                    bm_msg_t rx_msg = { .frame_addr = &dfu_serial_ctx.rx_payload_buf[dfu_serial_ctx.rx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE], .frame_length = frame_length};
                    retval = k_msgq_put(dfu_serial_ctx.dfu_rx_queue, &rx_msg, K_NO_WAIT);
                    k_sem_give(dfu_serial_ctx.dfu_sem);

                    if (retval)
                    {
                        LOG_ERR("Message could not be added to Queue");
                    }

                    /* Update index for storing next DFU RX Payload */
                    dfu_serial_ctx.rx_payload_idx++;
                    if (dfu_serial_ctx.rx_payload_idx >= CONFIG_BM_HOST_DFU_NUM_FRAMES)
                    {
                        dfu_serial_ctx.rx_payload_idx = 0;
                    }
                }
                else
                {
                    LOG_ERR("DFU RX MessageQueue full, dropping message. Get faster!");
                }
                break;
            case BRISTLEMOUTH_GENERIC:
            case BRISTLEMOUTH_IEEE802154:
            default:
                LOG_ERR("Received a non-DFU message");
                break;
        }
        k_sem_give(&processing_sem);
    }
}

struct k_sem* bm_dfu_serial_get_sem(void)
{
    return &dfu_sem;
}

struct k_msgq* bm_dfu_serial_get_tx_msgq_handler(void)
{
    return &dfu_tx_queue;
}

struct k_msgq* bm_dfu_serial_get_rx_msgq_handler(void)
{
    return &dfu_rx_queue;
}

/* Computes Bristlemouth Packet CRC16, stores in Tx Frame Buffer, and adds message to Queue for Tx Task  */
int bm_dfu_serial_frm_put(bristlemouth_frame_t* bm_frm)
{
    int retval = -1;
    uint16_t frame_length = bm_frm->frm_hdr.payload_length + sizeof(bristlemouth_frame_header_t);

    if (CONFIG_BM_MAX_FRAME_SIZE < frame_length + sizeof(bristlemouth_crc_t))
    {
        LOG_ERR("Frame is too large. Ignoring");
        goto out;
    }

    /* Computed on the entire packet, using CCITT */
    uint16_t computed_crc16 = crc16_ccitt(0, (uint8_t *) bm_frm, frame_length);

    if (k_msgq_num_free_get(&dfu_tx_queue))
    {
        /* Add frame to TX Contiguous Mem */
        memcpy(&dfu_serial_ctx.tx_payload_buf[dfu_serial_ctx.tx_payload_idx * CONFIG_BM_MAX_FRAME_SIZE], bm_frm, frame_length);
        /* Add CRC16 after Frame */
        memcpy(&dfu_serial_ctx.tx_payload_buf[(dfu_serial_ctx.tx_payload_idx  * CONFIG_BM_MAX_FRAME_SIZE) + frame_length], &computed_crc16, sizeof(bristlemouth_crc_t));

        /* Update the frame length with CRC16 */
        frame_length += sizeof(bristlemouth_crc_t);

        /* Add msg to TX Message Queue (for TX Task to consume */ 
        bm_msg_t tx_msg = { .frame_addr = &dfu_serial_ctx.tx_payload_buf[dfu_serial_ctx.tx_payload_idx  * CONFIG_BM_MAX_FRAME_SIZE], .frame_length = frame_length};
        retval = k_msgq_put(&dfu_tx_queue, &tx_msg, K_FOREVER);

        /* Update index for storing next TX Payload */
        dfu_serial_ctx.tx_payload_idx++;
        if (dfu_serial_ctx.tx_payload_idx >= CONFIG_BM_DFU_NUM_FRAMES)
        {
            dfu_serial_ctx.tx_payload_idx = 0;
        }
    }
    else
    {
        LOG_ERR("TX MessageQueue full, dropping message!");
    }
out:
    return retval;	
}

static int bm_dfu_serial_init( const struct device *arg )
{
    ARG_UNUSED(arg);

    k_tid_t thread_id;

    /* Init values for context parameters */
    dfu_serial_ctx.decode_buf_off = 0;
    dfu_serial_ctx.write_buf_idx = 0;
    dfu_serial_ctx.read_buf_idx = 1;
    dfu_serial_ctx.rx_payload_idx = 0;

    dfu_serial_ctx.serial_dev = device_get_binding(CONFIG_BM_DFU_SERIAL_DEV_NAME);

    if (dfu_serial_ctx.serial_dev != NULL) 
    {
        uart_irq_callback_set(dfu_serial_ctx.serial_dev, bm_dfu_serial_isr);
        uart_irq_rx_enable(( const struct device* ) dfu_serial_ctx.serial_dev);
    }

    thread_id = k_thread_create(&dfu_rx_thread_data, bm_dfu_rx_stack,
            K_THREAD_STACK_SIZEOF(bm_dfu_rx_stack),
            (k_thread_entry_t)bm_dfu_serial_rx_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

    if (thread_id == NULL)
    {
        LOG_ERR("BM Serial RX thread not created?");
        return -1;
    }

    thread_id = k_thread_create(&dfu_tx_thread_data, bm_dfu_tx_stack,
            K_THREAD_STACK_SIZEOF(bm_dfu_tx_stack),
            (k_thread_entry_t)bm_dfu_serial_tx_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

    if (thread_id == NULL)
    {
        LOG_ERR("BM Serial TX thread not created?");
        return -1;
    }

    dfu_serial_ctx.dfu_rx_queue = bm_dfu_get_serial_transport_service_queue();
    dfu_serial_ctx.dfu_sem = &dfu_sem;
    return 0;
}

SYS_INIT( bm_dfu_serial_init, POST_KERNEL, 2 );
