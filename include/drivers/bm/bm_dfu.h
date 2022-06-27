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

enum BM_DFU_BM_FRM_TYPE
{
    BM_DFU_START        = 0,
    BM_DFU_PAYLOAD_REQ  = 1,
    BM_DFU_PAYLOAD      = 2,
    BM_DFU_END          = 3,
    BM_DFU_ACK          = 4,
};

typedef struct bm_dfu_frame_header_t
{
    uint8_t frame_type;
} bm_dfu_frame_header_t;

typedef struct bm_dfu_event_update_request_t
{
    uint32_t image_size;
    uint16_t chunk_size;
    uint16_t crc16;
} bm_dfu_event_update_request_t;

typedef struct bm_dfu_event_image_chunk_t 
{
    uint16_t seq_num;
} bm_dfu_event_image_chunk_t;

typedef struct bm_dfu_event_ack_nack_received_t
{
} bm_dfu_event_ack_nack_received_t;

typedef struct bm_dfu_event_ack_timeout_t
{
} bm_dfu_event_ack_timeout_t;

typedef struct bm_dfu_event_cancel_update_t
{
} bm_dfu_event_cancel_update_t;

typedef struct bm_dfu_event_t
{
	uint8_t	type;
	union
    {
		bm_dfu_event_update_request_t	    update_request;
        bm_dfu_event_image_chunk_t          img_chunk;
        bm_dfu_event_ack_nack_received_t    ack_nack_received;
		bm_dfu_event_ack_timeout_t	        ack_timeout;
        bm_dfu_event_cancel_update_t	    cancel_update;
	} event;
} bm_dfu_event_t;

enum BM_DFU_EVT_TYPE
{
    DFU_EVENT_NONE              = 0,
    DFU_EVENT_INIT_SUCCESS      = 1,
    DFU_EVENT_UPDATE_REQUEST    = 2,
    DFU_EVENT_IMAGE_CHUNK       = 3,
    DFU_EVENT_FINAL_CHUNK       = 4,
    DFU_EVENT_ACK_NACK_RECEIVED = 5,
    DFU_EVENT_ACK_TIMEOUT       = 6,
    DFU_EVENT_CANCEL_UPDATE     = 7,
};

struct k_msgq* bm_dfu_get_transport_service_queue(void);

int bm_dfu_init( const struct device *arg );

#endif /* ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_H_ */