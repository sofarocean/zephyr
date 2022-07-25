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
#include <net/ieee802154_radio.h>
#include <storage/flash_map.h>
#include <sys/crc.h>

#include <zcbor_common.h>
#include <zcbor_decode.h>
#include <zcbor_encode.h>

#include "bm_zcbor_decode.h"
#include "bm_zcbor_encode.h"

#define LOG_MODULE_NAME bm_test
LOG_MODULE_REGISTER(bm_serial_test, LOG_LEVEL_DBG);

static struct k_thread _rx_thread_data;
K_THREAD_STACK_DEFINE( _rx_stack, 1024);

uint8_t tx_buf[259];
//uint8_t init_msg[] = "Hi from Device #1";
bm_frame_header_t frm_hdr = {BM_V0, BM_IEEE802154, NULL};

static void _rx_thread(void)
{
    struct k_msgq* rx_queue = NULL;
    bm_msg_t msg;
    uint16_t frame_length;
    uint16_t payload_length;
    int retval;

    struct Pet pet;
	size_t decode_len;

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
            retval = cbor_decode_Pet(&msg.frame_addr[sizeof(bm_frame_header_t)], payload_length, &pet, &decode_len);

            if (retval)
            {
                LOG_ERR("CBOR Decode error");
            }
            else
            {
                LOG_INF("Num pets: %d", pet._Pet_name_names_count);
                LOG_INF("Pet name 1: %.*s", pet._Pet_name_names[0].len, pet._Pet_name_names[0].value);
                LOG_INF("Pet name 2: %.*s", pet._Pet_name_names[1].len, pet._Pet_name_names[1].value);
            }
        }
    }
}

void main(void)
{
    LOG_INF( "Testing the BM Serial driver" );

    int retval;
    uint8_t output[25];
	size_t out_len;

    struct Pet pet = 
    {
        ._Pet_name_names = {{.value = "fido", .len = 4}, {.value = "arfer", .len = 5}},
        ._Pet_name_names_count = 2,
        ._Pet_birthday = {.value = (uint8_t[]){1,2,3,4,5,6,7,8}, .len = 8},
        ._Pet_species_choice = _Pet_species_dog
    };

    retval = cbor_encode_Pet(output, sizeof(output), &pet, &out_len);

    if (retval)
    {
        LOG_ERR("CBOR encoding error");
    }

    frm_hdr.payload_length = sizeof(output);

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
    memcpy(&tx_buf[sizeof(bm_frame_header_t)], output, sizeof(output));
    bm_frame_t *bm_frm = (bm_frame_t *)tx_buf;

    k_thread_create(&_rx_thread_data, _rx_stack,
        K_THREAD_STACK_SIZEOF(_rx_stack),
        (k_thread_entry_t) _rx_thread,
        NULL, NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);

    while (1)
    {
        usleep(1000000UL);
        retval = bm_serial_frm_put(bm_frm, BM_END_DEVICE);

        if (retval)
        {
            LOG_ERR( "TX MessageQueue is full, dropping message!");
        }
    }
} 