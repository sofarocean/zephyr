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
#include <drivers/bm/bm_serial.h>
#include <drivers/bm/bm_dfu.h>
#include <drivers/bm/bm_dfu_host.h>
#include <storage/flash_map.h>
#include <sys/crc.h>

#define LOG_MODULE_NAME bristlemouth_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

static uint32_t img_size = 0x3D0C;
static uint16_t max_chunk_size = 254;
static uint16_t num_chunks = 0;
static const struct flash_area *fa;

static int chunk_req_cb(uint16_t chunk_num, uint16_t *chunk_len, uint8_t *buf, uint16_t buf_len)
{
    int retval = 0;
    *chunk_len = 254;

    if (chunk_num == 61)
    {
        *chunk_len = img_size - (chunk_num * max_chunk_size);
    }

    if (buf_len < max_chunk_size)
    {
        LOG_ERR("Buffer provided too small");
        retval = -1;
        goto out;
    }
    uint32_t img_flash_off = (max_chunk_size * chunk_num);
    flash_area_read(fa, img_flash_off, buf, *chunk_len);
out:
    return retval;
}

void main(void)
{
    LOG_INF( "Performing DFU over BM serial");

    /* Open the secondary image slot */
    if (flash_area_open(FLASH_AREA_ID(image_1), &fa))
    {
        LOG_ERR("Flash Area could not be opened!\n");
        return;
    }

    bm_dfu_register_cb(chunk_req_cb);
}