/** @file
 * @brief Bristlemouth Serial driver
 *
 * A Bristlemouth DFU driver to send/receive firmware update messages over bm_serial.c
 */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
#include <kernel.h>
#include <storage/flash_map.h>
#include <dfu/mcuboot.h>
#include "bootutil/bootutil_public.h"
#include <sys/reboot.h>

#include <drivers/console/bm_serial.h>
#include <drivers/bm/bm_dfu.h>
#include <drivers/bm/bm_common.h>
#include <sys/crc.h>

LOG_MODULE_REGISTER(bm_dfu, CONFIG_UART_CONSOLE_LOG_LEVEL);

static struct k_thread rx_dfu_thread_data;

/* RX/TX Threads and associated message Queues */
K_THREAD_STACK_DEFINE(bm_dfu_rx_stack, CONFIG_BM_DFU_TASK_STACK_SIZE);
K_MSGQ_DEFINE(dfu_rx_queue, sizeof(bm_msg_t), CONFIG_BM_DFU_NUM_FRAMES, 4);

/* Firmware Update variables */
static uint32_t _img_length = 0;
static uint8_t _img_page_buf[BM_IMG_PAGE_LENGTH] = {0};
static uint16_t _img_page_byte_counter = 0;
static uint32_t _img_flash_offset = 0;
static bool _img_update_started = false;

static uint16_t remaining_page_length = 0;
static uint16_t dfu_frame_len = 0;
static uint8_t dfu_pad_counter = 0;
static const struct flash_area *fa;

/* Semaphore for receiving DFU ACK */
K_SEM_DEFINE(dfu_rx_sem, 0, 1);

static void bm_dfu_send_ack(void)
{
    int ret;
    bm_frame_header_t frm_hdr;
    bm_frame_t *ack_frm;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + sizeof(uint32_t)];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = 1;

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_ACK;

    ack_frm = (bm_frame_t *)tx_buf;
    ret = bm_serial_frm_put(ack_frm);

    if (ret)
    {
        LOG_ERR("ACK not sent");
    }
}

static int bm_dfu_process_ack(void)
{
    k_sem_give(&dfu_rx_sem);
    return 0;
}

static int bm_dfu_process_start(uint16_t man_decode_len, uint8_t * man_decode_buf)
{
    LOG_INF("DFU Start Received");
    int retval = 0;

    /* Account for BM Frame Header size and BM_DFU payload type */
    dfu_frame_len = man_decode_len - sizeof(bm_frame_header_t) - sizeof(bm_dfu_frame_header_t);
    bm_util_linear_memcpy(( uint8_t * ) &_img_length, \
                            &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], dfu_frame_len);

    _img_flash_offset = 0;
    _img_page_byte_counter = 0;
    memset(_img_page_buf, 0 , sizeof(_img_page_buf));

    /* Open the secondary image slot */
    retval = flash_area_open(FLASH_AREA_ID(image_1), &fa);
    if (retval)
    {
        LOG_ERR("Flash driver was not found!\n");
        goto out;
    }
    else
    {
        /* Erase memory in secondary image slot */
        retval = boot_erase_img_bank(FLASH_AREA_ID(image_1));
        if (retval)
        {
            LOG_ERR("Unable to erase Secondary Image slot");
            goto out;
        }
        else
        {
            _img_update_started = true;
            bm_dfu_send_ack();
        }
    }

out:
    return retval;
}

static int bm_dfu_process_payload(uint16_t man_decode_len, uint8_t * man_decode_buf)
{
    int retval = 0;

    LOG_INF("DFU Payload Received");

    if (_img_update_started)
    {
        /* Account for BM Frame Header size and BM_DFU payload type*/
        dfu_frame_len = man_decode_len - sizeof(bm_frame_header_t) - sizeof(bm_dfu_frame_header_t);

        if ( BM_IMG_PAGE_LENGTH > (dfu_frame_len + _img_page_byte_counter))
        {
            bm_util_linear_memcpy(&_img_page_buf[_img_page_byte_counter], \
                                 &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], dfu_frame_len);
            _img_page_byte_counter += dfu_frame_len;

            if (_img_page_byte_counter == BM_IMG_PAGE_LENGTH)
            {
                _img_page_byte_counter = 0;

                /* Perform page write and increment flash byte counter */
                retval = flash_area_write(fa, _img_flash_offset, &_img_page_buf, BM_IMG_PAGE_LENGTH);
                if (retval)
                {
                    LOG_ERR("Unable to write DFU frame to Flash");
                    goto out;
                }
                else
                {
                    _img_flash_offset += BM_IMG_PAGE_LENGTH;
                }
            }
            bm_dfu_send_ack();
        }
        else
        {
            remaining_page_length = BM_IMG_PAGE_LENGTH - _img_page_byte_counter;
            bm_util_linear_memcpy(&_img_page_buf[_img_page_byte_counter], \
                                    &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], remaining_page_length);
            _img_page_byte_counter += remaining_page_length;
            
            if (_img_page_byte_counter == BM_IMG_PAGE_LENGTH)
            {
                _img_page_byte_counter = 0;

                /* Perform page write and increment flash byte counter */
                retval = flash_area_write(fa, _img_flash_offset, &_img_page_buf, BM_IMG_PAGE_LENGTH);
                if (retval)
                {
                    LOG_ERR("Unable to write DFU frame to Flash");
                    goto out;
                }
                else
                {
                    _img_flash_offset += BM_IMG_PAGE_LENGTH;
                }
            }
            
            /* Memcpy the remaining bytes to next page */
            bm_util_linear_memcpy(&_img_page_buf[_img_page_byte_counter], \
                                    &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t) + remaining_page_length], \
                                    (dfu_frame_len - remaining_page_length) );
            _img_page_byte_counter += (dfu_frame_len - remaining_page_length);
            bm_dfu_send_ack();
        }
    }
    else
    {
        LOG_ERR("Bristlemouth Firmware Update has not started");
    }

out:
    return retval;
}

static int bm_dfu_process_end(void)
{
    int retval = 0;

    LOG_INF("DFU End Received");

    /* If there are any dirty bytes, write to flash */
    if (_img_page_byte_counter != 0)
    {
        dfu_pad_counter = 0;

        /* STM32L4 needs flash writes to be 8-byte aligned */
        while ( (_img_page_byte_counter + dfu_pad_counter) % 8 )
        {
            dfu_pad_counter++;
            _img_page_buf[_img_page_byte_counter + dfu_pad_counter] = 0;
        }

        /* Perform page write and increment flash byte counter */
        retval = flash_area_write(fa, _img_flash_offset, &_img_page_buf, (_img_page_byte_counter + dfu_pad_counter));
        if (retval)
        {
            LOG_ERR("Unable to write DFU frame to Flash");
            goto out;
        }
        else
        {
            _img_flash_offset += _img_page_byte_counter;
        }
    }

    /* Send last ACK before staging firmware update and reseting device */
    bm_dfu_send_ack();

    if (_img_update_started && (_img_length == _img_flash_offset))
    {
        /* Set as temporary switch. New application must confirm or else MCUBoot will
           switch back to old image */
        boot_set_pending(0);
        sys_reboot(0);
    }
    else
    {
        LOG_ERR("Bristlemouth Firmware Update mismatch in bytes");
    }

out:
    return retval;
}

/**
 * BM DFU RX Thread
 */
static void bm_dfu_rx_thread(void)
{
    bm_msg_t msg;
    uint16_t payload_length;
    uint8_t payload_type;

    LOG_DBG("BM DFU RX thread started");

    while (1)
    {
        k_msgq_get(&dfu_rx_queue, &msg, K_FOREVER);

        payload_length = msg.frame_length;
        payload_type = msg.frame_addr[sizeof(bm_frame_header_t)];

        switch (payload_type)
        {
            case BM_DFU_ACK:
                bm_dfu_process_ack();
                break;
            case BM_DFU_START:
                bm_dfu_process_start(payload_length, (uint8_t *) (msg.frame_addr));
                break;
            case BM_DFU_PAYLOAD:
                bm_dfu_process_payload(payload_length, (uint8_t *) (msg.frame_addr));
                break;
            case BM_DFU_END:
                bm_dfu_process_end();
                break;
            default:
                break;
        }
    }
}

struct k_msgq* bm_dfu_init(void)
{
    k_thread_create(&rx_dfu_thread_data, bm_dfu_rx_stack,
            K_THREAD_STACK_SIZEOF(bm_dfu_rx_stack),
            (k_thread_entry_t)bm_dfu_rx_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

    return &dfu_rx_queue;
}

struct k_sem* bm_dfu_get_rx_sem(void)
{
    return &dfu_rx_sem;
}
