/** @file
 *  @brief Bristlemouth Serial driver header file.
 *
 *  Bristlemouth Serial driver that allows applications to handle all aspects of
 *  received protocol data.
 */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BRISTLEMOUTH_MANCHESTER_H_
#define ZEPHYR_INCLUDE_DRIVERS_BRISTLEMOUTH_MANCHESTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const uint16_t BM_MAN_ENCODE_TABLE[256];
extern const int8_t BM_MAN_DECODE_TABLE[256];

#ifdef __cplusplus
}
#endif

#endif // ZEPHYR_INCLUDE_DRIVERS_BRISTLEMOUTH_MANCHESTER_H_