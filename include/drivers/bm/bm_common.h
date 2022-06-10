/** @file
 *  @brief Bristlemouth common header file.
 *
 *  Bristlemouth common API for serial and DFU
 */

/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BM_BM_COMMON_H_
#define ZEPHYR_INCLUDE_DRIVERS_BM_BM_COMMON_H_

typedef struct bm_msg_t
{
    uint16_t            frame_length;
    volatile uint8_t*   frame_addr; 
} bm_msg_t;

void bm_util_linear_memcpy(uint8_t* dest, uint8_t* src, size_t n);

#endif /* ZEPHYR_INCLUDE_DRIVERS_BM_BM_COMMON_H_ */