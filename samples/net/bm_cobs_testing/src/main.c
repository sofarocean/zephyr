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

#include <drivers/gpio.h>
#include <net/ieee802154_radio.h>
#include <drivers/ieee802154/ieee802154_bm_serial.h>

#define LOG_MODULE_NAME bristlemouth_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#define APP_BANNER "***** OpenThread NCP on Zephyr %s *****"

static const struct device *radio_dev;
struct ieee802154_config cfg;
struct net_pkt tx_pkt;
static struct net_buf tx_payload;

void _sample_handler(const struct device *dev, enum ieee802154_event evt,
			void *event_params)
{
    LOG_INF("we reached this handler");
}

void main(void)
{
    LOG_INF( "Testing the UPIPE COBS Decoding/Encoding functionality" );
    bm_upipe_start();
    cfg.event_handler = _sample_handler;

    radio_dev = device_get_binding(CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);

    while (1)
    {
        bm_upipe_tx(radio_dev, IEEE802154_TX_MODE_DIRECT, &tx_pkt, &tx_payload);
        sleep(5);
    }
}