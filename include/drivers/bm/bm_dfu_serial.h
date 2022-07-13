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
#include "bm_common.h"

#ifdef __cplusplus
extern "C" {
#endif

struct k_msgq* bm_dfu_serial_get_tx_msgq_handler(void);

#endif /* ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_DFU_SERIAL_H_ */