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
#include <init.h>
#include <logging/log.h>
#include <storage/flash_map.h>
#include <bootutil/bootutil_public.h>
#include <dfu/mcuboot.h>
#include <sys/reboot.h>

#include <drivers/bm/bm_serial.h>
#include <drivers/bm/bm_dfu.h>
#include <drivers/bm/bm_common.h>
#include <sys/crc.h>

LOG_MODULE_REGISTER(bm_dfu, CONFIG_BM_LOG_LEVEL);

static struct k_thread _rx_dfu_thread_data;

/* RX/TX Threads and associated message Queues */
K_THREAD_STACK_DEFINE(_bm_dfu_rx_stack, CONFIG_BM_DFU_TASK_STACK_SIZE);
K_MSGQ_DEFINE(_dfu_rx_queue, sizeof(bm_msg_t), CONFIG_BM_DFU_NUM_FRAMES, 4);

/* Semaphore for receiving DFU ACK */
K_SEM_DEFINE(_dfu_rx_sem, 0, 1);

/* Firmware Update variables */
static uint32_t _img_length = 0;
static uint8_t  _img_page_buf[BM_IMG_PAGE_LENGTH] = {0};
static uint16_t _img_page_byte_counter = 0;
static uint32_t _img_flash_offset = 0;
static bool     _img_update_started = false;

static uint16_t _remaining_page_length = 0;
static uint16_t _dfu_frame_len = 0;
static uint8_t  _dfu_pad_counter = 0;

static const struct flash_area *_fa;

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
    k_sem_give(&_dfu_rx_sem);
    return 0;
}

static int bm_dfu_process_start(uint16_t man_decode_len, uint8_t* man_decode_buf)
{
    LOG_INF("DFU Start Received");
    int retval = 0;

    /* Account for BM Frame Header size and BM_DFU payload type */
    _dfu_frame_len = man_decode_len - sizeof(bm_frame_header_t) - sizeof(bm_dfu_frame_header_t);
    memcpy( (uint8_t*)&_img_length, &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], _dfu_frame_len);

    _img_flash_offset = 0;
    _img_page_byte_counter = 0;
    memset(_img_page_buf, 0 , sizeof(_img_page_buf));

    /* Open the secondary image slot */
    retval = flash_area_open(FLASH_AREA_ID(image_1), &_fa);
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
        _dfu_frame_len = man_decode_len - sizeof(bm_frame_header_t) - sizeof(bm_dfu_frame_header_t);

        if ( BM_IMG_PAGE_LENGTH > (_dfu_frame_len + _img_page_byte_counter))
        {
            memcpy(&_img_page_buf[_img_page_byte_counter], \
                                 &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], _dfu_frame_len);
            _img_page_byte_counter += _dfu_frame_len;

            if (_img_page_byte_counter == BM_IMG_PAGE_LENGTH)
            {
                _img_page_byte_counter = 0;

                /* Perform page write and increment flash byte counter */
                retval = flash_area_write(_fa, _img_flash_offset, &_img_page_buf, BM_IMG_PAGE_LENGTH);
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
            _remaining_page_length = BM_IMG_PAGE_LENGTH - _img_page_byte_counter;
            memcpy(&_img_page_buf[_img_page_byte_counter], \
                                    &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], _remaining_page_length);
            _img_page_byte_counter += _remaining_page_length;
            
            if (_img_page_byte_counter == BM_IMG_PAGE_LENGTH)
            {
                _img_page_byte_counter = 0;

                /* Perform page write and increment flash byte counter */
                retval = flash_area_write(_fa, _img_flash_offset, &_img_page_buf, BM_IMG_PAGE_LENGTH);
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
            memcpy(&_img_page_buf[_img_page_byte_counter], \
                                    &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t) + _remaining_page_length], \
                                    (_dfu_frame_len - _remaining_page_length) );
            _img_page_byte_counter += (_dfu_frame_len - _remaining_page_length);
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
        _dfu_pad_counter = 0;

        /* STM32L4 needs flash writes to be 8-byte aligned */
        while ( (_img_page_byte_counter + _dfu_pad_counter) % 8 )
        {
            _dfu_pad_counter++;
            _img_page_buf[_img_page_byte_counter + _dfu_pad_counter] = 0;
        }

        /* Perform page write and increment flash byte counter */
        retval = flash_area_write(_fa, _img_flash_offset, &_img_page_buf, (_img_page_byte_counter + _dfu_pad_counter));
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


// TODO: 
// - Change to be a service thread - handle RX, TX, and state machine here
// - Encapsulate RX functionality into "handle_messages" 
// TODO: Create callbacks for events/errors/state changes
// TODO: Create APIs to allow the application to approve/deny image transfers and kick over
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
        k_msgq_get(&_dfu_rx_queue, &msg, K_FOREVER);

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

// TODO: get dfu rx queue method
struct k_msgq* bm_dfu_get_rx_queue(void)
{
    return &_dfu_rx_queue;
}

// TODO: What is this for?
struct k_sem* bm_dfu_get_rx_sem(void)
{
    return &_dfu_rx_sem;
}

int bm_dfu_init( const struct device *arg )
{
    ARG_UNUSED(arg);

    k_thread_create(&_rx_dfu_thread_data, _bm_dfu_rx_stack,
            K_THREAD_STACK_SIZEOF(_bm_dfu_rx_stack),
            (k_thread_entry_t)bm_dfu_rx_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

    // TODO: Handle potential errors properly
    return 0;
}

SYS_INIT( bm_dfu_init, POST_KERNEL, 1 );