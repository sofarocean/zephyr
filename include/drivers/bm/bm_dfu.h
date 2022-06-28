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
#include <smf.h>

enum BM_DFU_ERR_TYPE
{
    BM_DFU_ERR_NONE         = 0,
    BM_DFU_ERR_TOO_LARGE    = 1,
    BM_DFU_ERR_SAME_VER     = 2,
    BM_DFU_ERR_MISMATCH_LEN = 3,
    BM_DFU_ERR_BAD_CRC      = 4,
};

enum BM_DFU_BM_FRM_TYPE
{
    BM_DFU_START        = 0,
    BM_DFU_PAYLOAD_REQ  = 1,
    BM_DFU_PAYLOAD      = 2,
    BM_DFU_END          = 3,
    BM_DFU_ACK          = 4,
    BM_DFU_ABORT        = 5,
    BM_DFU_HEARTBEAT    = 6,
};

typedef struct bm_dfu_img_info_t
{
    uint32_t image_size;
    uint16_t chunk_size;
    uint16_t crc16;
    uint8_t major_ver;
    uint8_t minor_ver;
} bm_dfu_img_info_t;

typedef struct bm_dfu_frame_header_t
{
    uint8_t frame_type;
} bm_dfu_frame_header_t;

typedef struct bm_dfu_event_init_success_t
{
    uint8_t reserved;
}bm_dfu_event_init_success_t;

typedef struct bm_dfu_event_begin_update_t
{
    uint8_t reserved;
}bm_dfu_event_begin_update_t;

typedef struct bm_dfu_event_update_request_t
{
    bm_dfu_img_info_t img_info;   
} bm_dfu_event_update_request_t;

typedef struct bm_dfu_event_chunk_request_t 
{
    uint16_t seq_num;
} bm_dfu_event_chunk_request_t;

typedef struct bm_dfu_event_image_chunk_t 
{
    uint8_t* payload_buf;
    uint16_t payload_length;    
} bm_dfu_event_image_chunk_t;

typedef struct bm_dfu_event_update_end_t 
{
    uint8_t success;
    uint8_t err_code;
} bm_dfu_event_update_end_t;

typedef struct bm_dfu_event_ack_received_t
{
    uint8_t success;
    uint8_t err_code;
} bm_dfu_event_ack_received_t;

typedef struct bm_dfu_event_ack_timeout_t
{
    uint8_t reserved;
} bm_dfu_event_ack_timeout_t;

typedef struct bm_dfu_event_chunk_timeout_t
{
    uint8_t reserved;
} bm_dfu_event_chunk_timeout_t;

typedef struct bm_dfu_event_heartbeat_timeout_t
{
    uint8_t reserved;
} bm_dfu_event_heartbeat_timeout_t;

typedef struct bm_dfu_event_heartbeat_t
{
    uint8_t reserved;
} bm_dfu_event_heartbeat_t;

typedef struct bm_dfu_event_abort_t
{
    uint8_t reserved;
} bm_dfu_event_abort_t;

typedef struct bm_dfu_event_t
{
	uint8_t	type;
	union
    {
        bm_dfu_event_init_success_t         init_success;
        bm_dfu_event_begin_update_t         begin_update;
		bm_dfu_event_update_request_t	    update_request;
        bm_dfu_event_chunk_request_t        chunk_request;
        bm_dfu_event_image_chunk_t          img_chunk;
        bm_dfu_event_update_end_t           update_end;
        bm_dfu_event_ack_received_t         ack_received;
		bm_dfu_event_ack_timeout_t	        ack_timeout;
        bm_dfu_event_chunk_timeout_t        chunk_timeout;
        bm_dfu_event_heartbeat_timeout_t    heartbeat_timeout;
        bm_dfu_event_heartbeat_timeout_t    heartbeat;
        bm_dfu_event_abort_t	            abort;
	} event;
} bm_dfu_event_t;

enum BM_DFU_EVT_TYPE
{
    DFU_EVENT_NONE                      = 0,
    DFU_EVENT_INIT_SUCCESS              = 1,
    DFU_EVENT_BEGIN_UPDATE              = 2,
    DFU_EVENT_RECEIVED_UPDATE_REQUEST   = 3,
    DFU_EVENT_CHUNK_REQUEST             = 4,
    DFU_EVENT_IMAGE_CHUNK               = 5,
    DFU_EVENT_UPDATE_END                = 6,
    DFU_EVENT_ACK_RECEIVED              = 7,
    DFU_EVENT_ACK_TIMEOUT               = 8,
    DFU_EVENT_CHUNK_TIMEOUT             = 9,
    DFU_EVENT_HEARTBEAT_TIMEOUT         = 10,
    DFU_EVENT_HEARTBEAT                 = 11,
    DFU_EVENT_ABORT                     = 12,
};

#define BM_DFU_MAX_CHUNK_RETRIES    3
#define BM_IMG_PAGE_LENGTH          2048


typedef struct dfu_core_ctx_t
{
    struct smf_ctx ctx;
    bm_dfu_event_t current_event;
} dfu_core_ctx_t;

enum BM_DFU_HFSM_STATES 
{ 
    BM_DFU_STATE_INIT               = 0, 
    BM_DFU_STATE_IDLE               = 1, 
    BM_DFU_STATE_CLIENT             = 2,
    BM_DFU_STATE_HOST               = 3, 
    BM_DFU_STATE_ERROR              = 4,
    BM_DFU_STATE_CLIENT_RECEIVING   = 5, 
    BM_DFU_STATE_CLIENT_VALIDATING  = 6, 
    BM_DFU_STATE_CLIENT_ACTIVATING  = 7,
    BM_DFU_STATE_HOST_REQ_UPDATE    = 8,
    BM_DFU_STATE_HOST_UPDATE        = 9,
};

struct k_msgq* bm_dfu_get_subsystem_queue(void);
struct k_msgq* bm_dfu_get_transport_service_queue(void);
bm_dfu_event_t bm_dfu_get_current_event(void);
void bm_dfu_set_state(uint8_t state);

#endif /* ZEPHYR_INCLUDE_DRIVERS_BM_BM_DFU_H_ */