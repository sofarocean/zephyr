/** @file
 *  @brief Bristlemouth DFU Client driver header file.
 *
 *  Bristlemouth DFU Client driver that controls the state entry/exit functions
 *  for the Client States of the HFSM
 */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_CLIENT_H_
#define ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_CLIENT_H_

#include <kernel.h>
#include <drivers/bm/bm_dfu.h>

#define BM_DFU_MAX_CHUNK_RETRIES    3
#define BM_DFU_CLIENT_CHUNK_TIMEOUT 9000000UL

typedef struct dfu_client_ctx_t
{
    struct k_msgq* dfu_subystem_queue;
    struct k_sem* dfu_sem;
    /* Variables from DFU Start */
    uint32_t image_size;
    uint16_t num_chunks;
    uint16_t crc16;
    /* Variables from DFU Payload */
    uint8_t chunk_buf[CONFIG_BM_MAX_FRAME_SIZE];
    uint16_t chunk_length;
    /* Flash Mem variables */
    const struct flash_area *fa;
    uint16_t img_page_byte_counter;
    uint32_t img_flash_offset;
    uint8_t img_page_buf[BM_IMG_PAGE_LENGTH];
    /* Chunk variables */
    uint8_t chunk_retry_num;
    uint16_t current_chunk;
    struct k_timer chunk_timer;
} dfu_client_ctx_t;

struct dfu_client_ctx_t* bm_dfu_client_get_context(void);
void bm_dfu_client_process_request(void);

/* HFSM functions */
void s_client_entry(void *o);
void s_client_exit(void *o);
void s_client_receiving_entry(void *o);
void s_client_receiving_run(void *o);
void s_client_validating_entry(void *o);
void s_client_activating_entry(void *o);

#endif /* ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_CLIENT_H_ */