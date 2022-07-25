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

void main(void)
{
    LOG_INF( "Performing DFU over BM serial");
}