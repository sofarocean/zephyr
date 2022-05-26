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
#include <kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

enum BM_VERSION 
{
    BM_V0 = 0,
    BM_V1 = 1,
};

enum BM_PAYLOAD_TYPE
{
    BM_GENERIC      = 0,
    BM_IEEE802154   = 1,
    BM_DFU_START    = 2,
    BM_DFU_PAYLOAD  = 3,
    BM_DFU_END      = 4,
    BM_DFU_ACK      = 5,
};

typedef enum BM_PARSE_STATE
{
    BM_ALIGN,
    BM_COLLECT_HEADER,
    BM_COLLECT_PAYLOAD,
} bm_parse_state_t;

typedef struct bm_ctx_t
{
    const struct device* serial_dev;
    struct k_sem sem;
    struct k_timer timer;
    uint32_t interframe_delay_us;
    uint8_t buf[8*CONFIG_BM_MAX_FRAME_SIZE];
} bm_ctx_t;

typedef struct bm_parse_ret_t
{
    bm_parse_state_t new_state;
    uint8_t success;
} bm_parse_ret_t;

typedef struct bm_ret_t
{
    int         retval;
    uint16_t    length;
    uint8_t*    buf_ptr;
} bm_ret_t;

typedef struct bm_rx_t
{
    uint16_t    length;
    uint8_t     buf[(2*CONFIG_BM_MAX_FRAME_SIZE) + CONFIG_BM_PREAMBLE_LEN];
} bm_rx_t;

typedef struct bm_msg_t
{
    uint16_t            frame_length;
    volatile uint8_t*   frame_addr; 
} bm_msg_t;
 
typedef uint16_t bm_crc_t;

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

#define BM_IMG_PAGE_LENGTH          2048
typedef uint32_t bm_img_length_t;

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

/** @brief Process Serial Byte for Bristlemouth Protocol.
 *
 *  This function is used to process the incoming byte and control
 *  the driver's state machine
 * 
 *  @param byte Incoming byte from serial interface
 *
 *  @return Struct containing success, buf_ptr, and length
 */
uint16_t bm_serial_process_byte(uint8_t* byte, uint16_t num_bytes);


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
