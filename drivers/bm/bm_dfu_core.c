/** @file
 * @brief Bristlemouth Serial driver
 *
 * A Bristlemouth DFU driver to send/receive firmware update messages over bm_serial.c
 */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <init.h>
#include <logging/log.h>
#include <storage/flash_map.h>
#include <bootutil/bootutil_public.h>
#include <dfu/mcuboot.h>
#include <sys/reboot.h>

#include <drivers/bm/bm_serial.h>
#include <drivers/bm/bm_dfu.h>
#include <drivers/bm/bm_common.h>
#include <smf.h>
#include <kernel.h>

LOG_MODULE_REGISTER(bm_dfu, CONFIG_BM_LOG_LEVEL);

static struct k_thread _subsystem_dfu_thread_data;
static struct k_thread _transport_service_dfu_thread_data;

K_THREAD_STACK_DEFINE(_bm_dfu_subsystem_stack, CONFIG_BM_DFU_TASK_STACK_SIZE);
K_MSGQ_DEFINE(_dfu_subsystem_queue, sizeof(bm_dfu_event_t), CONFIG_BM_DFU_NUM_FRAMES, 4);

K_THREAD_STACK_DEFINE(_bm_dfu_transport_service_stack, CONFIG_BM_DFU_TASK_STACK_SIZE);
K_MSGQ_DEFINE(_dfu_transport_service_queue, sizeof(bm_msg_t), CONFIG_BM_DFU_NUM_FRAMES, 4);

/* Forward declaration of state table */
static const struct smf_state dfu_states[];

enum BM_DFU_HFSM_STATES { BM_DFU_STATE_INIT, BM_DFU_STATE_IDLE, BM_DFU_STATE_CLIENT,
                          BM_DFU_STATE_HOST, BM_DFU_STATE_ERROR,
                          BM_DFU_STATE_CLIENT_RECEIVING, BM_DFU_STATE_CLIENT_VALIDATING,
                          BM_DFU_STATE_CLIENT_ERROR, BM_DFU_STATE_CLIENT_ACTIVATING };

typedef struct dfu_ctx_t
{
    struct smf_ctx ctx;
    /* Event just received */
    bm_dfu_event_t current_event;
    /* Variables from DFU Start */
    uint32_t image_size;
    uint16_t num_chunks;
    uint16_t crc16;
    /* Variables from DFU Payload */
    uint8_t * frame_addr;
    uint16_t frame_length;
    /* Flash Mem variables */
    struct flash_area *fa;
    uint16_t img_page_byte_counter;
    uint32_t img_flash_offset;
    uint8_t img_page_buf[BM_IMG_PAGE_LENGTH];
    /* Other */
    uint16_t current_chunk;
    struct k_timer ack_timer;
} dfu_ctx_t;

static dfu_ctx_t _dfu_context;

static int bm_dfu_process_payload(uint16_t man_decode_len, uint8_t * man_decode_buf)
{
    int retval = 0;

    /* Account for BM Frame Header size and BM_DFU payload type*/
    uint16_t _dfu_frame_len = man_decode_len - sizeof(bm_frame_header_t) - sizeof(bm_dfu_frame_header_t);

    if ( BM_IMG_PAGE_LENGTH > (_dfu_frame_len + _dfu_context.img_page_byte_counter))
    {
        memcpy(&_dfu_context.img_page_buf[_dfu_context.img_page_byte_counter], \
                                &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], _dfu_frame_len);
        _dfu_context.img_page_byte_counter += _dfu_frame_len;

        if (_dfu_context.img_page_byte_counter == BM_IMG_PAGE_LENGTH)
        {
            _dfu_context.img_page_byte_counter = 0;

            /* Perform page write and increment flash byte counter */
            retval = flash_area_write(_fa, _dfu_context.img_flash_offset, &_dfu_context.img_page_buf, BM_IMG_PAGE_LENGTH);
            if (retval)
            {
                LOG_ERR("Unable to write DFU frame to Flash");
                goto out;
            }
            else
            {
                _dfu_context.img_flash_offset += BM_IMG_PAGE_LENGTH;
            }
        }
    }
    else
    {
        uint16_t _remaining_page_length = BM_IMG_PAGE_LENGTH - _dfu_context.img_page_byte_counter;
        memcpy(&_dfu_context.img_page_buf[_dfu_context.img_page_byte_counter], \
                                &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], _remaining_page_length);
        _dfu_context.img_page_byte_counter += _remaining_page_length;
        
        if (_dfu_context.img_page_byte_counter == BM_IMG_PAGE_LENGTH)
        {
            _dfu_context.img_page_byte_counter = 0;

            /* Perform page write and increment flash byte counter */
            retval = flash_area_write(_dfu_context.fa, _dfu_context.img_flash_offset, &_dfu_context.img_page_buf, BM_IMG_PAGE_LENGTH);
            if (retval)
            {
                LOG_ERR("Unable to write DFU frame to Flash");
                goto out;
            }
            else
            {
                _dfu_context.img_flash_offset += BM_IMG_PAGE_LENGTH;
            }
        }
        
        /* Memcpy the remaining bytes to next page */
        memcpy(&_dfu_context.img_page_buf[_dfu_context.img_page_byte_counter], \
                                &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t) + _remaining_page_length], \
                                (_dfu_frame_len - _remaining_page_length) );
        _dfu_context.img_page_byte_counter += (_dfu_frame_len - _remaining_page_length);
    }
out:
    return retval;
}

static void bm_dfu_send_ack(void)
{
    int ret;
    bm_frame_header_t frm_hdr;
    bm_frame_t *ack_frm;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + 1];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = 1;

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_ACK;

    ack_frm = (bm_frame_t *)tx_buf;
    ret = bm_serial_frm_put(ack_frm);

    if (ret)
    {
        LOG_ERR("ACK not sent");
    }
}

static int bm_dfu_process_end(void)
{
    int retval = 0;

    /* If there are any dirty bytes, write to flash */
    if (_dfu_context.img_page_byte_counter != 0)
    {
        uint8_t _dfu_pad_counter = 0;

        /* STM32L4 needs flash writes to be 8-byte aligned */
        while ( (_dfu_context.img_page_byte_counter + _dfu_pad_counter) % 8 )
        {
            _dfu_pad_counter++;
            _dfu_context.img_page_buf[_dfu_context.img_page_byte_counter + _dfu_pad_counter] = 0;
        }

        /* Perform page write and increment flash byte counter */
        retval = flash_area_write(_dfu_context.fa, _dfu_context.img_flash_offset, &_dfu_context.img_page_buf, (_dfu_context.img_page_byte_counter + _dfu_pad_counter));
        if (retval)
        {
            LOG_ERR("Unable to write DFU frame to Flash");
            goto out;
        }
        else
        {
            _dfu_context.img_flash_offset += _dfu_context.img_page_byte_counter;
        }
    }
out:
    return retval;
}

static void bm_dfu_req_next_chunk(void)
{
    bm_frame_header_t frm_hdr;
    bm_frame_t *chunk_req_frm;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + 2];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = 2;

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_PAYLOAD_REQ;
    tx_buf[sizeof(bm_frame_header_t) + 1] = _dfu_context.current_chunk;
    chunk_req_frm = (bm_frame_t *)tx_buf;

    if (bm_serial_frm_put(chunk_req_frm))
    {
        LOG_ERR("Chunk Request not sent");
    }
}

static void ack_timer_handler(struct k_timer *tmr)
{
    bm_dfu_event_t evt;
    int retval;

    evt.type = DFU_EVENT_ACK_NACK_RECEIVED;
    retval = k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT);
}

/* 
State machine entry, run and exit functions 
*/

static void s_init_run(void *o)
{
    if (_dfu_context.current_event.type == DFU_EVENT_INIT_SUCCESS)
    {
        smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_IDLE]);
    }
}

static void s_idle_run(void *o)
{
    uint32_t image_size;
    uint16_t chunk_size;
    if (_dfu_context.current_event.type == DFU_EVENT_UPDATE_REQUEST)
    {
        image_size = _dfu_context.current_event.event.update_request.image_size;
        chunk_size = _dfu_context.current_event.event.update_request.chunk_size;
        /* TODO: Is there a way to grab the partition size from the .dts file? */
        if (image_size <= 0x67000)
        {
            _dfu_context.image_size = image_size;
            _dfu_context.num_chunks = ( image_size / chunk_size ) + 1;
            _dfu_context.crc16 = _dfu_context.current_event.event.update_request.crc16;
            smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_CLIENT_RECEIVING]);
        }
    }
}

static void s_client_entry(void *o)
{
    /* Open the secondary image slot */
    if (flash_area_open(FLASH_AREA_ID(image_1), &_dfu_context.fa))
    {
        LOG_ERR("Flash driver was not found!\n");
    }
    else
    {
        /* Erase memory in secondary image slot */
        if (boot_erase_img_bank(FLASH_AREA_ID(image_1)))
        {
            LOG_ERR("Unable to erase Secondary Image slot");
        }
        else
        {
            bm_dfu_send_ack();
        }
    }
}

static void s_client_receiving_entry(void *o)
{
    /* Start from Chunk #0 */
    _dfu_context.current_chunk = 0;

    /* Request Next Chunk */
    bm_dfu_req_next_chunk();

    /* Kickoff ACK timeout */
    k_timer_start((struct k_timer*) &_dfu_context.ack_timer, K_USEC(1000000UL), K_NO_WAIT);
}

static void s_client_receiving_run(void *o)
{
    uint8_t ack_received = 0;
    /* Request next packet from Host */ 

    if ( _dfu_context.current_event.type == DFU_EVENT_ACK_NACK_RECEIVED)
    {
        ack_received = 1; 
    }
    else if (_dfu_context.current_event.type == DFU_EVENT_IMAGE_CHUNK && ack_received)
    {
        /* Before getting next chunk */
        ack_received = 0;

        /* Process the frame */
        if (bm_dfu_process_payload(_dfu_context.frame_length, _dfu_context.frame_addr))
        {
            /* TODO: We are unable to write to Flash. What do we do? */
            smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_CLIENT_ERROR]);
        }

        /* Request Next Chunk */
        _dfu_context.current_chunk++;
        bm_dfu_req_next_chunk();

    }
    else if (_dfu_context.current_event.type == DFU_EVENT_FINAL_CHUNK)
    {
        /* Process the frame */
        if (bm_dfu_process_end())
        {
            /* TODO: We are unable to write to Flash. What do we do? */
            smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_CLIENT_ERROR]);
        }
        else
        {
            /* Received what we think is the full image */
            smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_CLIENT_VALIDATING]);
        }
    }
}

static void s_client_validating_entry(void *o)
{
    /* Verify image length */
    if (_dfu_context.image_size != _dfu_context.img_flash_offset))
    {
        smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_CLIENT_ERROR]);
    }
    else
    {
        /* Verify CRC. If ok, then move to */

    }
}

static void s_client_activating_entry(void *o)
{
    /* Set as temporary switch. New application must confirm or else MCUBoot will
    switch back to old image */
    boot_set_pending(0);
    sys_reboot(0);
}

/* Populate state table */
static const struct smf_state dfu_states[] = 
{
        /* Parent states does not have a run action */
        [BM_DFU_STATE_INIT] = SMF_CREATE_STATE(NULL, s_init_run, NULL, NULL),
        [BM_DFU_STATE_IDLE] = SMF_CREATE_STATE(NULL, s_idle_run, NULL, NULL),
        [BM_DFU_STATE_CLIENT] = SMF_CREATE_STATE(s_client_entry, NULL, NULL, NULL]),
        [BM_DFU_STATE_HOST] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL),
        [BM_DFU_STATE_ERROR] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL),

        /* Child states of BM_DFU_STATE_CLIENT */ 
        [BM_DFU_STATE_CLIENT_RECEIVING] = SMF_CREATE_STATE(s_client_receiving_entry, s_client_receiving_run, NULL, &dfu_states[BM_DFU_STATE_CLIENT]),
        [BM_DFU_STATE_CLIENT_VALIDATING] = SMF_CREATE_STATE(s_client_validating_entry, NULL, NULL, &dfu_states[BM_DFU_STATE_CLIENT]),
        [BM_DFU_STATE_CLIENT_ERROR] = SMF_CREATE_STATE(NULL, NULL, NULL, &dfu_states[BM_DFU_STATE_CLIENT]),
        [BM_DFU_STATE_CLIENT_ACTIVATING] = SMF_CREATE_STATE(s_client_activating_entry, NULL, NULL, &dfu_states[BM_DFU_STATE_CLIENT]),
};

static void bm_dfu_transport_service_thread(void)
{
    bm_msg_t msg;
    uint16_t payload_length;
    uint8_t payload_type;
    bm_dfu_event_t evt;
    int retval;

    LOG_DBG("BM DFU Transport Service thread started");

    while (1)
    {
        k_msgq_get(&_dfu_transport_service_queue, &msg, K_FOREVER);

        payload_type = msg.frame_addr[sizeof(bm_frame_header_t)];

        switch (payload_type)
        {
            case BM_DFU_ACK:
                k_timer_stop(&_dfu_context.ack_timer);
                evt.type = DFU_EVENT_ACK_NACK_RECEIVED;
                retval = k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT);
                if (retval)
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_START:
                evt.type = DFU_EVENT_UPDATE_REQUEST;
                memcpy( (uint8_t*) &evt.event.update_request, &msg.frame_addr[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], sizeof(bm_dfu_event_update_request_t));
                retval = k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT);
                if (retval)
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_PAYLOAD:
                _dfu_context.frame_length = msg.frame_length;
                _dfu_context.frame_addr = msg.frame_addr;
                evt.type = DFU_EVENT_IMAGE_CHUNK;
                retval = k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT);
                if (retval)
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_END:
                evt.type = DFU_EVENT_FINAL_CHUNK;
                retval = k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT);
                if (retval)
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                //bm_dfu_process_end();
                break;
            default:
                break;
        }

    }
}

// TODO: 
// - Change to be a service thread - handle RX, TX, and state machine here
// - Encapsulate RX functionality into "handle_messages" 
// TODO: Create callbacks for events/errors/state changes
// TODO: Create APIs to allow the application to approve/deny image transfers and kick over
/**
 * BM DFU Subsystem Thread (Runs HFSM)
 */
static void bm_dfu_subsystem_thread(void)
{
    int ret;

    LOG_DBG("BM DFU Subsystem thread started");

    while (1)
    {
        _dfu_context.current_event.type = DFU_EVENT_NONE;
        k_msgq_get(&_dfu_subsystem_queue, &_dfu_context.current_event, K_FOREVER);

        ret = smf_run_state(SMF_CTX(&_dfu_context));
        if (ret) 
        {
            /* handle return code and terminate state machine */
            LOG_ERR("State Machine Error: terminating");
            break;
        }
    }
}

// TODO: get dfu rx queue method
struct k_msgq* bm_dfu_get_transport_service_queue(void)
{
    return &_dfu_transport_service_queue;
}

int bm_dfu_init( const struct device *arg )
{
    k_tid_t thread_id;
    bm_dfu_event_t evt;
    int retval;

    ARG_UNUSED(arg);

    /* Set initial state */
    smf_set_initial(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_INIT]);

    /* Initialize ACK Timer */
    k_timer_init(( struct k_timer* ) &_dfu_context.ack_timer, ack_timer_handler, NULL); 

    thread_id = k_thread_create(&_transport_service_dfu_thread_data, _bm_dfu_transport_service_stack,
            K_THREAD_STACK_SIZEOF(_bm_dfu_transport_service_stack),
            (k_thread_entry_t)bm_dfu_transport_service_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

    if (thread_id == NULL)
    {
        LOG_ERR("DFU Transport Service thread not created");
        return -1;
    }

    thread_id = k_thread_create(&_subsystem_dfu_thread_data, _bm_dfu_subsystem_stack,
            K_THREAD_STACK_SIZEOF(_bm_dfu_subsystem_stack),
            (k_thread_entry_t)bm_dfu_subsystem_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

    if (thread_id)
    {
        evt.type = DFU_EVENT_INIT_SUCCESS;
        retval = k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT);
        if (retval)
        {
            LOG_ERR("Message could not be added to Queue");
        }
        return 0;
    }
    else
    {
        LOG_ERR("DFU Subsystem thread not created");
        return -1;
    }
}

SYS_INIT( bm_dfu_init, POST_KERNEL, 1 );