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

#ifndef ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_DFU_SERIAL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_DFU_SERIAL_H_

#include <stdlib.h>
#include <stdint.h>
#include <kernel.h>
#include <zephyr/bm/bm_common.h>
#include <zephyr/bm/manchester.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dfu_serial_ctx_t
{
    struct k_msgq* dfu_rx_queue;
    struct k_sem* dfu_sem;
    volatile uint16_t decode_buf_off;
    volatile uint8_t write_buf_idx;
    volatile uint8_t read_buf_idx;
    const struct device* serial_dev;
    bristlemouth_rx_t cobs_decoding_buf[2];
    uint8_t rx_payload_buf[CONFIG_BM_HOST_DFU_NUM_FRAMES * CONFIG_BM_MAX_FRAME_SIZE];
    uint8_t rx_payload_idx;
    uint8_t tx_payload_buf[CONFIG_BM_HOST_DFU_NUM_FRAMES * CONFIG_BM_MAX_FRAME_SIZE];
    uint8_t tx_payload_idx;
} dfu_serial_ctx_t;

struct k_sem* bm_dfu_serial_get_sem(void);
struct k_msgq* bm_dfu_serial_get_tx_msgq_handler(void);
struct k_msgq* bm_dfu_serial_get_rx_msgq_handler(void);
int bm_dfu_serial_frm_put(bristlemouth_frame_t* bm_frm);

#endif /* ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_DFU_SERIAL_H_ */