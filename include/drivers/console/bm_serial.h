/** @file
 *  @brief Bristlemouth Serial driver header file.
 *
 *  Bristlemouth Serial driver that allows applications to handle all aspects of
 *  received protocol data.
 */

/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_SERIAL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_SERIAL_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_BM_FRAMES           20
#define MAX_BM_FRAME_SIZE       255
#define MAX_ENCODED_BUF_SIZE	258
#define TASK_STACK_SIZE         1024

#define MAX_SERIAL_DEV_COUNT    3

enum BM_VERSION 
{
	BM_V0 = 0,
	BM_V1 = 1,
};

enum BM_PAYLOAD_TYPE
{
	BM_GENERIC      = 0,
	BM_IEEE802154   = 1,
};

typedef struct bm_decoded_t
{
    uint16_t    length;
    uint8_t     buf[MAX_BM_FRAME_SIZE];
} bm_decoded_t;

typedef struct bm_msg_t
{
    uint16_t    frame_length;
    uint8_t*    frame_addr; 
} bm_msg_t;
 
typedef struct bm_frame_header_t
{
    uint8_t         version;
    uint8_t         payload_type;
    uint16_t        payload_length;
} bm_frame_header_t;

typedef struct bm_frame_t
{
    bm_frame_header_t   frm_hdr;
    uint8_t             payload[];
} bm_frame_t;

typedef uint16_t bm_crc_t;

/** @brief Received data callback.
 *
 *  This function is called when new data is received over BM Serial. The off parameter
 *  can be used to alter offset at which received data is stored. Typically,
 *  when the complete data is received and a new buffer is provided off should
 *  be set to 0.
 *
 *  @param buf Buffer with received data.
 *  @param off Data offset on next received and accumulated data length.
 *
 *  @return Buffer to be used on next receive.
 */
typedef uint8_t *(*bm_serial_recv_cb)(uint8_t *buf, size_t *off);

/** @brief Init Bristlemouth Serial application.
 *
 *  This function is used to initialize the Serial RX/TX threads
 *  and register an RX callback
 *
 */
void bm_serial_init(void);

/** @brief Send a frame over Bristlemouth Serial.
 *
 *  This function computes a CRC16 and sends a frame over BM Serial.
 *
 *  @param bm_frm Bristlemouth Frame with Header and Payload
 *
 *  @return 0 on success or negative error
 */
int bm_serial_frm_put(bm_frame_t* bm_frm);

/** @brief Get RX Message Queue
 *
 *  This function gets the RX Message Queue listened by ieee802154_bm_serial.c
 *
 *  @return RX Message Queue used by bm_serial.c
 */
struct k_msgq* bm_serial_get_rx_msgq_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_SERIAL_H_ */
