/* main.c - OpenThread NCP */

/*
 * Copyright (c) 2020 Tridonic GmbH & Co KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <logging/log.h>

#include <drivers/gpio.h>
#include <net/ieee802154_radio.h>
#include <drivers/console/bm_serial.h>
#include <storage/flash_map.h>

#define LOG_MODULE_NAME bristlemouth_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

static struct k_thread sample_rx_thread;
K_THREAD_STACK_DEFINE(sample_rx_stack, 1024);

/* Semaphore for ACKs */
K_SEM_DEFINE(ack_sem, 0, 1);

static void _rx_thread(void)
{
	struct k_msgq* sample_rx_queue = NULL;
	bm_msg_t msg;
    uint8_t bm_type;

    while (sample_rx_queue == NULL)
	{
		sample_rx_queue = bm_serial_get_rx_msgq_handler();
	} 

    while (1)
	{
		k_msgq_get(sample_rx_queue, &msg, K_FOREVER);
		bm_type = ((bm_frame_header_t*) msg.frame_addr)->payload_type;

		if (bm_type == BM_DFU_ACK)
		{
            k_sem_give(&ack_sem);
        }
	}
}

void main(void)
{
    uint32_t remaining_bytes;
    static const struct flash_area *fa;
    uint32_t img_flash_off = 0;
    bm_frame_t *bm_frm;

    LOG_INF( "Performing DFU over BM serial");

	bm_serial_init();

	k_thread_create(&sample_rx_thread, sample_rx_stack,
		K_THREAD_STACK_SIZEOF(sample_rx_stack),
		(k_thread_entry_t)_rx_thread,
		NULL, NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);

    /* Signed Blinky App is 17280 bytes (0x4380) written to Secondary Image Slot at 0x73000.
       Make sure that Blinky image is written to Slot 1 before running this app */
    uint8_t tx_buf[259];

    /* First send DFU Start */
    bm_frame_header_t dfu_start_frm_hdr = {BM_V0, BM_DFU_START, sizeof(bm_img_length_t)};
    bm_img_length_t img_length = 0x4380; /* TODO, make this configurable in prj.conf */
    remaining_bytes = img_length;

    memcpy(tx_buf, &dfu_start_frm_hdr, sizeof(bm_frame_header_t));
	memcpy(&tx_buf[sizeof(bm_frame_header_t)], (uint8_t *) &img_length, sizeof(bm_img_length_t));

    bm_frm = (bm_frame_t *)tx_buf;
    int ret = bm_serial_frm_put(bm_frm);

    if (ret)
    {
        LOG_ERR("Unable to schedule frame to be sent");   
    }

    k_sem_take(&ack_sem, K_FOREVER);

    bm_frame_header_t dfu_payload_frm_hdr = {BM_V0, BM_DFU_PAYLOAD, 0};

    /* Open the secondary image slot */
    ret = flash_area_open(FLASH_AREA_ID(image_1), &fa);
    if (ret)
    {
        LOG_ERR("Flash driver was not found!\n");
        return;
    }

    /* Now send image chunk by chunk */
    while (remaining_bytes != 0)
    {
        if (remaining_bytes > 255)
        {
            dfu_payload_frm_hdr.payload_length = 255;
        }
        else
        {
            dfu_payload_frm_hdr.payload_length = remaining_bytes;
        }
        remaining_bytes -= dfu_payload_frm_hdr.payload_length;
        memcpy(tx_buf, &dfu_payload_frm_hdr, sizeof(bm_frame_header_t));

        ret = flash_area_read(fa, img_flash_off,
			      &tx_buf[sizeof(bm_frame_header_t)], dfu_payload_frm_hdr.payload_length);
        img_flash_off += dfu_payload_frm_hdr.payload_length;

        bm_frm = (bm_frame_t *)tx_buf;
        ret = bm_serial_frm_put(bm_frm);

        if (ret)
        {
            LOG_ERR("Unable to schedule frame to be sent"); 
        }

        k_sem_take(&ack_sem, K_FOREVER);
    }

    /* Send DFU_END */
    bm_frame_header_t dfu_end_frm_hdr = {BM_V0, BM_DFU_END, sizeof(bm_img_length_t)};

    memcpy(tx_buf, &dfu_end_frm_hdr, sizeof(bm_frame_header_t));
	memcpy(&tx_buf[sizeof(bm_frame_header_t)], (uint8_t *) &img_length, sizeof(bm_img_length_t));

    bm_frm = (bm_frame_t *)tx_buf;
    ret = bm_serial_frm_put(bm_frm);

    if (ret)
    {
        LOG_ERR("ACK not sent");
    }

    k_sem_take(&ack_sem, K_FOREVER);

}