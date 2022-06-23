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
    uint16_t   frame_length;
    uint8_t*   frame_addr; 
} bm_msg_t;

#endif /* ZEPHYR_INCLUDE_DRIVERS_BM_BM_COMMON_H_ */