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
#include <drivers/console/bm_serial.h>

#define LOG_MODULE_NAME bm_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#define USER0_NODE DT_ALIAS(user0)
#define USER1_NODE DT_ALIAS(user1)

/* These map to D3/D4 on the L496ZG Nucleo*/
static const struct gpio_dt_spec user0 = GPIO_DT_SPEC_GET(USER0_NODE, gpios);
static const struct gpio_dt_spec user1 = GPIO_DT_SPEC_GET(USER1_NODE, gpios);

void main(void)
{
    LOG_INF( "Testing the BM Serial driver" );

    /* Checking/Setting up GPIO Debug Pins */
    if (!device_is_ready(user0.port))
    {
        LOG_ERR("GPIO 0 Port not ready");
    }

    if (!device_is_ready(user1.port))
    {
        LOG_ERR("GPIO 1 Port not ready");
    }

    int ret = gpio_pin_configure_dt(&user0, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) 
    {
        LOG_ERR("GPIO 0 unable to be configured\n");
    }

    ret = gpio_pin_configure_dt(&user1, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) 
    {
        LOG_ERR("GPIO 1 unable to be configured\n");
    }

	bm_serial_init();

    uint8_t tx_buf[32];
    uint8_t msg[] = "Hello World";
    int retval;
    bm_frame_header_t frm_hdr = {BM_V0, BM_IEEE802154, sizeof(msg)};

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	memcpy(&tx_buf[sizeof(bm_frame_header_t)], msg, frm_hdr.payload_length);
    bm_frame_t *bm_frm = (bm_frame_t *)tx_buf;

	while (1)
    {
        retval = bm_serial_frm_put(bm_frm);
	    if (retval)
	    {
		    LOG_ERR( "TX MessageQueue is full, dropping message!");
	    }
        usleep(100000UL);
    }
}