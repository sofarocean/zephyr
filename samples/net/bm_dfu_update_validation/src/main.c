/* main.c - OpenThread NCP */

/*
 * Copyright (c) 2020 Tridonic GmbH & Co KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <drivers/bm/bm_serial.h>
#include <drivers/bm/bm_dfu.h>
#include <bootutil/bootutil_public.h>

#define LOG_MODULE_NAME bm_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

void main(void)
{
    LOG_INF( "This is the new image!" );
    boot_set_confirmed();
    bm_dfu_update_end(BM_END_DEVICE, 1, BM_DFU_ERR_NONE);

    while (1)
    {
        usleep(1000000UL);
    }
}