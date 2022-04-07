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
#include <drivers/console/bm_serial.h>

#define LOG_MODULE_NAME bristlemouth_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

static struct k_thread sample_rx_thread;
K_THREAD_STACK_DEFINE(sample_rx_stack, TASK_STACK_SIZE);
static struct k_thread sample_tx_thread;
K_THREAD_STACK_DEFINE(sample_tx_stack, TASK_STACK_SIZE);

#define IS_PRODUCER 1

#define USER0_NODE DT_ALIAS(user0)
#define USER1_NODE DT_ALIAS(user1)

/* These map to D3/D4 on the L496ZG Nucleo*/
static const struct gpio_dt_spec user0 = GPIO_DT_SPEC_GET(USER0_NODE, gpios);
static const struct gpio_dt_spec user1 = GPIO_DT_SPEC_GET(USER1_NODE, gpios);

static void _tx_thread(void)
{
    static uint8_t counter = 0;
    int retval;
    uint8_t tx_buf[140];
    uint8_t msg[] = "loWorldHelloWorldHelloWorldHelloWorldHelloWorldHelloWorldHelloWorldHelloWorldHelloWorldHelloWorldHelloWorldHelloWorldWeAreDone";
    bm_frame_header_t frm_hdr = {BM_V0, BM_GENERIC, sizeof(msg)};

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	memcpy(&tx_buf[sizeof(bm_frame_header_t)], msg, frm_hdr.payload_length);

     while (1)
    {
        /* add a counter value for rx to watch for */
        memcpy(&tx_buf[sizeof(bm_frame_header_t)], &counter, sizeof(counter));
        bm_frame_t *bm_frm = (bm_frame_t *)tx_buf;
        counter++;

        retval = bm_serial_frm_put(bm_frm);
	    if (retval)
	    {
		    LOG_ERR( "TX MessageQueue is full, dropping message!");
	    }
        usleep(5000UL);
    }
}

static void _rx_thread(void)
{
	struct k_msgq* sample_rx_queue = NULL;
	bm_msg_t msg;
	uint16_t frame_length;
	uint16_t payload_length;
    static uint8_t last_val = 0;
    uint8_t current_val;
    static uint8_t first = 1;

    while (sample_rx_queue == NULL)
	{
		sample_rx_queue = bm_serial_get_rx_msgq_handler();
	} 

    while (1)
	{
		// Wait on bm_serial.c RX Thread to put message on queue
		k_msgq_get(sample_rx_queue, &msg, K_FOREVER);
		frame_length = msg.frame_length;
		payload_length = frame_length - sizeof(bm_frame_header_t);
		uint8_t bm_version = ((bm_frame_header_t*) msg.frame_addr)->version;

		if (bm_version != BM_GENERIC)
		{
			LOG_ERR("Incompatible version. Discarding Frame");
			continue;
		}

        current_val = (uint8_t) *(msg.frame_addr + sizeof(bm_frame_header_t));
        if (first)
        {
            first = 0;
        }
        else
        {
            if (current_val == 0 && last_val == 255)
            {
                LOG_INF("wrapped around");
            }
            else if (current_val != (last_val +1))
            {
                LOG_ERR("C: %d L: %d", current_val, last_val);
                gpio_pin_toggle_dt(&user1);
            }
            else
            {
                gpio_pin_toggle_dt(&user0);
            }
        }
        last_val = current_val;
	}
}

void main(void)
{
    LOG_INF( "Testing the UPIPE COBS Decoding/Encoding functionality" );

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

	k_thread_create(&sample_rx_thread, sample_rx_stack,
		K_THREAD_STACK_SIZEOF(sample_rx_stack),
		(k_thread_entry_t)_rx_thread,
		NULL, NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);

#if IS_PRODUCER== 1
    k_thread_create(&sample_tx_thread, sample_tx_stack,
		K_THREAD_STACK_SIZEOF(sample_tx_stack),
		(k_thread_entry_t)_tx_thread,
		NULL, NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);
#endif
}