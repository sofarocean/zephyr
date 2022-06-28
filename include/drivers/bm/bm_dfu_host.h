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

#ifndef ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_HOST_H_
#define ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_HOST_H_

#include <kernel.h>
#include <drivers/bm/bm_dfu.h>

#define BM_DFU_MAX_ACK_RETRIES          3
#define BM_DFU_HOST_ACK_TIMEOUT         5000000UL
#define BM_DFU_HOST_HEARTBEAT_TIMEOUT   9000000UL

typedef int (*bm_dfu_chunk_req_cb)(uint16_t chunk_num, uint16_t *chunk_len, uint8_t *buf, uint16_t buf_len);

typedef struct dfu_host_ctx_t
{
    struct k_msgq* dfu_subystem_queue;
    struct k_timer ack_timer;
    uint8_t ack_retry_num;
    struct k_timer heartbeat_timer;
    bm_dfu_img_info_t img_info;
    bm_dfu_chunk_req_cb req_cb;
    uint8_t chunk_buf[CONFIG_BM_MAX_FRAME_SIZE];
    uint16_t chunk_length;
} dfu_host_ctx_t;

void s_host_entry(void *o);
void s_host_exit(void *o);
void s_host_req_update_entry(void *o);
void s_host_req_update_run(void *o);
void s_host_update_entry(void *o);
void s_host_update_run(void *o);
void bm_dfu_host_start_update(bm_dfu_img_info_t *img_info, bm_dfu_chunk_req_cb req_cb);

#endif /* ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_HOST_H_ */