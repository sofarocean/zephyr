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
#include <drivers/bm/bm_common.h>
#include <drivers/bm/bm_dfu.h>
#include <storage/flash_map.h>
#include <sys/crc.h>

#define LOG_MODULE_NAME bristlemouth_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

struct k_sem* _dfu_rx_sem = NULL;

void main(void)
{
    uint32_t remaining_bytes;
    static const struct flash_area *fa;
    uint32_t img_flash_off = 0;
    bm_frame_t *bm_frm;
    uint16_t chunk_size = 0;

    LOG_INF( "Performing DFU over BM serial");

	bm_serial_init();


    while (_dfu_rx_sem == NULL)
	{
		_dfu_rx_sem = bm_dfu_get_rx_sem();
	} 

    /* Signed Blinky App is 15628 bytes (0x3D0C) written to Secondary Image Slot at 0x73000.
       Make sure that Blinky image is written to Slot 1 before running this app */
    uint8_t tx_buf[259];

    /* First send DFU Start */
    bm_frame_header_t dfu_start_frm_hdr = {BM_V0, BM_DFU, sizeof(bm_img_length_t) + sizeof(bm_dfu_frame_header_t)};
    bm_img_length_t img_length = 0x3D0C; /* TODO, make this configurable in prj.conf */
    remaining_bytes = img_length;

    memcpy(tx_buf, &dfu_start_frm_hdr, sizeof(bm_frame_header_t));
    tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_START;
	memcpy(&tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], (uint8_t *) &img_length, sizeof(bm_img_length_t));

    bm_frm = (bm_frame_t *)tx_buf;
    int ret = bm_serial_frm_put(bm_frm);

    if (ret)
    {
        LOG_ERR("Unable to schedule frame to be sent");   
    }

    k_sem_take(_dfu_rx_sem, K_FOREVER);

    bm_frame_header_t dfu_payload_frm_hdr = {BM_V0, BM_DFU, 0};

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
        if (remaining_bytes > 254)
        {
            chunk_size = 254;
        }
        else
        {
            chunk_size = remaining_bytes;
        }
        remaining_bytes -= chunk_size;

        /* account for bm_dfu message type */
        dfu_payload_frm_hdr.payload_length = (chunk_size + sizeof(bm_dfu_frame_header_t));

        memcpy(tx_buf, &dfu_payload_frm_hdr, sizeof(bm_frame_header_t));
        tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_PAYLOAD;

        ret = flash_area_read(fa, img_flash_off,
			      &tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], chunk_size);
        img_flash_off += chunk_size;

        bm_frm = (bm_frame_t *)tx_buf;
        ret = bm_serial_frm_put(bm_frm);

        if (ret)
        {
            LOG_ERR("Unable to schedule frame to be sent"); 
        }

        k_sem_take(_dfu_rx_sem, K_FOREVER);
    }

    /* Send DFU_END */
    bm_frame_header_t dfu_end_frm_hdr = {BM_V0, BM_DFU, sizeof(bm_dfu_frame_header_t)};

    memcpy(tx_buf, &dfu_end_frm_hdr, sizeof(bm_frame_header_t));
	tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_END; 

    bm_frm = (bm_frame_t *)tx_buf;
    ret = bm_serial_frm_put(bm_frm);

    if (ret)
    {
        LOG_ERR("ACK not sent");
    }

    k_sem_take(_dfu_rx_sem, K_FOREVER);

}