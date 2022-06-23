/** @file
 *  @brief Bristlemouth DFU driver header file.
 *
 *  Bristlemouth DFU driver that allows applications to send/receive
 *  firmware updates.
 */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_H_
#define ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_H_

#include <kernel.h>

enum BM_DFU_TYPE
{
    BM_DFU_START    = 0,
    BM_DFU_PAYLOAD  = 1,
    BM_DFU_END      = 2,
    BM_DFU_ACK      = 3,
};

typedef struct bm_dfu_frame_header_t
{
    uint8_t frame_type;
} bm_dfu_frame_header_t;

struct k_msgq* bm_dfu_get_rx_queue(void);
struct k_sem* bm_dfu_get_rx_sem(void);

int bm_dfu_init( const struct device *arg );

#endif /* ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_H_ */