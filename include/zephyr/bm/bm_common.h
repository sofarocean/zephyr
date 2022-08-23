/** @file
 *  @brief Bristlemouth common header file.
 *
 *  Bristlemouth common API for serial and DFU
 */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BM_BM_COMMON_H_
#define ZEPHYR_INCLUDE_DRIVERS_BM_BM_COMMON_H_

typedef struct bm_msg_t
{
    uint16_t    frame_length;
    uint8_t*    frame_addr;
} bm_msg_t;

enum BRISTLEMOUTH_VERSION 
{
    BRISTLEMOUTH_V0 = 0,
    BRISTLEMOUTH_V1 = 1,
};

enum BRISTLEMOUTH_PAYLOAD_TYPE
{
    BRISTLEMOUTH_GENERIC      = 0,
    BRISTLEMOUTH_IEEE802154   = 1,
    BRISTLEMOUTH_DFU          = 2,
};

typedef enum bristlemouth_parse_state_t
{
    BRISTLEMOUTH_ALIGN,
    BRISTLEMOUTH_COLLECT_HEADER,
    BRISTLEMOUTH_COLLECT_PAYLOAD,
} bristlemouth_parse_state_t;

typedef struct bristlemouth_rx_t
{
    uint16_t    length;
    uint8_t     buf[(2*CONFIG_BM_MAX_FRAME_SIZE) + CONFIG_BM_PREAMBLE_LEN];
} bristlemouth_rx_t;

typedef struct bristlemouth_ctx_t
{
    const struct device* serial_dev;
    struct k_sem sem;
    struct k_timer timer;
    uint32_t interframe_delay_us;
    uint8_t buf[8*CONFIG_BM_MAX_FRAME_SIZE];
    bristlemouth_rx_t encoded_rx_buf[2];
    volatile uint8_t write_buf_idx;
    volatile uint8_t read_buf_idx;
} bristlemouth_ctx_t;

typedef struct bristlemouth_parse_ret_t
{
    bristlemouth_parse_state_t new_state;
    uint8_t success;
} bristlemouth_parse_ret_t;

typedef struct bristlemouth_ret_t
{
    int         retval;
    uint16_t    length;
    uint8_t*    buf_ptr;
} bristlemouth_ret_t;

typedef uint16_t bristlemouth_crc_t;

typedef struct bristlemouth_frame_header_t
{
    uint8_t         version;
    uint8_t         payload_type;
    uint16_t        payload_length;
} bristlemouth_frame_header_t;

typedef struct bristlemouth_frame_t
{
    bristlemouth_frame_header_t   frm_hdr;
    uint8_t             payload[];
} bristlemouth_frame_t;

typedef uint32_t bristlemouth_img_length_t;

extern const uint16_t MAN_ENCODE_TABLE[256];
extern const int8_t MAN_DECODE_TABLE[256];

#endif /* ZEPHYR_INCLUDE_DRIVERS_BM_BM_COMMON_H_ */