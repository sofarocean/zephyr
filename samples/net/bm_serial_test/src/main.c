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
#include <sys/crc.h>
#include <logging/log.h>
#include <drivers/console/bm_serial.h>

#define LOG_MODULE_NAME bm_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

static struct k_thread _rx_thread_data;
K_THREAD_STACK_DEFINE( _rx_stack, TASK_STACK_SIZE);

static void _rx_thread(void)
{
	struct k_msgq* rx_queue = NULL;
	bm_msg_t msg;
	uint16_t frame_length;
	uint16_t payload_length;

	while (rx_queue == NULL)
	{
		rx_queue = bm_serial_get_rx_msgq_handler();
	} 

	while (1)
	{
		// Wait on bm_serial.c RX Thread to put message on queue
		k_msgq_get(rx_queue, &msg, K_FOREVER);

        frame_length = msg.frame_length;
		payload_length = frame_length - sizeof(bm_frame_header_t);
		uint8_t bm_payload_type = ((bm_frame_header_t*) msg.frame_addr)->payload_type;

		if (bm_payload_type != BM_IEEE802154)
		{
			LOG_ERR("Incompatible version. Discarding Frame");
			continue;
		}
        else
        {
            LOG_INF("%s", &msg.frame_addr[sizeof(bm_frame_header_t)]);
        }
    }
}

void main(void)
{
    LOG_INF( "Testing the BM Serial driver" );

	bm_serial_init();

    uint8_t tx_buf[32];
    uint8_t msg[] = "Hello World";
    int retval;
    bm_frame_header_t frm_hdr = {BM_V0, BM_IEEE802154, sizeof(msg)};
    frm_hdr.header_crc = crc16_ccitt(0, (uint8_t *) &frm_hdr, sizeof(bm_frame_header_t) - sizeof(bm_crc_t));

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	memcpy(&tx_buf[sizeof(bm_frame_header_t)], msg, frm_hdr.payload_length);
    bm_frame_t *bm_frm = (bm_frame_t *)tx_buf;

    k_thread_create(&_rx_thread_data, _rx_stack,
		K_THREAD_STACK_SIZEOF(_rx_stack),
		(k_thread_entry_t) _rx_thread,
		NULL, NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);

	while (1)
    {
        retval = bm_serial_frm_put(bm_frm);
        
	    if (retval)
	    {
		    LOG_ERR( "TX MessageQueue is full, dropping message!");
	    }
        usleep(2500UL);
    }
}