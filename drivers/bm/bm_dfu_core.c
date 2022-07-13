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
#include <drivers/bm/bm_dfu_client.h>
#include <drivers/bm/bm_dfu_host.h>
#include <drivers/bm/bm_common.h>
#include <kernel.h>

LOG_MODULE_REGISTER(bm_dfu, CONFIG_BM_LOG_LEVEL);

static struct k_thread _subsystem_dfu_thread_data;
static struct k_thread _transport_service_dfu_thread_data;

K_THREAD_STACK_DEFINE(_bm_dfu_subsystem_stack, CONFIG_BM_DFU_TASK_STACK_SIZE);
K_MSGQ_DEFINE(_dfu_subsystem_queue, sizeof(bm_dfu_event_t), CONFIG_BM_DFU_NUM_FRAMES, 4);

K_THREAD_STACK_DEFINE(_bm_dfu_transport_service_stack, CONFIG_BM_DFU_TASK_STACK_SIZE);
K_MSGQ_DEFINE(_dfu_transport_service_queue, sizeof(bm_msg_t), CONFIG_BM_DFU_NUM_FRAMES, 4);

static const struct smf_state dfu_states[];
static dfu_core_ctx_t _dfu_context;

/**
 * @brief Run Function for the Init State
 *
 * @note This state looks for success of bm_dfu_init 
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
static void s_init_run(void *o)
{
    if (_dfu_context.current_event.type == DFU_EVENT_INIT_SUCCESS)
    {
        smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_IDLE]);
    }
}

/**
 * @brief Run Function for the Idle State
 *
 * @note Depending on config parameters can move to client or host class states on DFU events 
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
static void s_idle_run(void *o)
{
    if (_dfu_context.current_event.type == DFU_EVENT_RECEIVED_UPDATE_REQUEST)
    {
#ifdef CONFIG_BM_DFU_CLIENT
        /* Client */
        bm_dfu_client_process_request();
#endif
    }
    else if (_dfu_context.current_event.type == DFU_EVENT_BEGIN_HOST)
    {
#ifdef CONFIG_BM_DFU_HOST
        /* Host */
        smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_HOST_REQ_UPDATE]);
#endif
    }
}

/**
 * @brief Entry Function for the Error State
 *
 * @note The Host processes the current error state and either proceeds to the IDLE state or stays in Error (fatal)
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
static void s_error_entry(void *o)
{
    char err_msg[50];
    switch (_dfu_context.error)
    {
        case BM_DFU_ERR_FLASH_ACCESS:
            strncpy(err_msg, "Flash access error (Fatal Error)", sizeof(err_msg));
            goto fatal;
        case BM_DFU_ERR_IMG_CHUNK_ACCESS:
            strncpy(err_msg, "Unable to get image chunk", sizeof(err_msg));
            goto nonfatal;
        case BM_DFU_ERR_TOO_LARGE:
            strncpy(err_msg, "Image too large for Client", sizeof(err_msg));
            goto nonfatal;
        case BM_DFU_ERR_SAME_VER:
            strncpy(err_msg, "Client already loaded with image", sizeof(err_msg));
            goto nonfatal;
        case BM_DFU_ERR_MISMATCH_LEN:
            strncpy(err_msg, "Length mismatch", sizeof(err_msg));
            goto nonfatal;
        case BM_DFU_ERR_BAD_CRC:
            strncpy(err_msg, "CRC mismatch", sizeof(err_msg));
            goto nonfatal;
        case BM_DFU_ERR_TIMEOUT:
            strncpy(err_msg, "DFU Timeout", sizeof(err_msg));
            goto nonfatal;
        case BM_DFU_ERR_BM_FRAME:
            strncpy(err_msg, "BM Processing Error", sizeof(err_msg));
            goto nonfatal;
        case BM_DFU_ERR_ABORTED:
            strncpy(err_msg, "BM Aborted Error", sizeof(err_msg));
            goto nonfatal;
        case BM_DFU_ERR_NONE:
        default:
            break;
    }

nonfatal:
    LOG_ERR("%s", err_msg);
    smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_IDLE]);
fatal:
    return;
}

/**
 * @brief Exit Function for the Error State
 *
 * @note Currently empty. Any cleanup from error state can go here
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
static void s_error_exit(void *o)
{
    /* TODO: Anything to do here? */
}

/* Hierarchical Finite State Machine outlined here: 
    https://lucid.app/lucidchart/25c8a78b-49d6-4f67-b30b-0b3f39434e51/edit?invitationId=inv_f1d747dc-6abc-43eb-a22d-099c5a8bdbdb# 
*/
static const struct smf_state dfu_states[] = 
{
        [BM_DFU_STATE_INIT] = SMF_CREATE_STATE(NULL, s_init_run, NULL, NULL),
        [BM_DFU_STATE_IDLE] = SMF_CREATE_STATE(NULL, s_idle_run, NULL, NULL),
        [BM_DFU_STATE_ERROR] = SMF_CREATE_STATE(s_error_entry, NULL, s_error_exit, NULL),

#ifdef CONFIG_BM_DFU_CLIENT
        /* Child states of BM_DFU_STATE_CLIENT */
        [BM_DFU_STATE_CLIENT] = SMF_CREATE_STATE(s_client_entry, NULL, s_client_exit, NULL),
        [BM_DFU_STATE_CLIENT_RECEIVING] = SMF_CREATE_STATE(s_client_receiving_entry, s_client_receiving_run, NULL, &dfu_states[BM_DFU_STATE_CLIENT]),
        [BM_DFU_STATE_CLIENT_VALIDATING] = SMF_CREATE_STATE(s_client_validating_entry, NULL, NULL, &dfu_states[BM_DFU_STATE_CLIENT]),
        [BM_DFU_STATE_CLIENT_ACTIVATING] = SMF_CREATE_STATE(s_client_activating_entry, NULL, NULL, &dfu_states[BM_DFU_STATE_CLIENT]),
#endif
#ifdef  CONFIG_BM_DFU_HOST
        /* Host states of BM_DFU_STATE_HOST */
        [BM_DFU_STATE_HOST] = SMF_CREATE_STATE(s_host_entry, NULL, s_host_exit, NULL),
        [BM_DFU_STATE_HOST_REQ_UPDATE] = SMF_CREATE_STATE(s_host_req_update_entry, s_host_req_update_run, NULL, &dfu_states[BM_DFU_STATE_HOST]),
        [BM_DFU_STATE_HOST_UPDATE] = SMF_CREATE_STATE(s_host_update_entry, s_host_update_run, NULL, &dfu_states[BM_DFU_STATE_HOST]),
#endif
};

/**
 * @brief BM DFU Transport Service Thread
 *
 * @note This thread takes BM Serial Frames from bm_serial.c's RX thread and translates them into 
 *       events for the DFU Hierarchical Finite State Machine
 * 
 * @return none
 */
static void bm_dfu_transport_service_thread(void)
{
    bm_msg_t msg;
    uint8_t payload_type;
    bm_dfu_event_t evt;

    LOG_DBG("BM DFU Transport Service thread started");

    while (1)
    {
        k_msgq_get(&_dfu_transport_service_queue, &msg, K_FOREVER);
        payload_type = msg.frame_addr[sizeof(bm_frame_header_t)];

        switch (payload_type)
        {
            case BM_DFU_START:
                LOG_INF("Received update request");
                evt.type = DFU_EVENT_RECEIVED_UPDATE_REQUEST;
                memcpy( (uint8_t*) &evt.event.update_request, &msg.frame_addr[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], sizeof(bm_dfu_event_update_request_t));
                if (k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT))
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_PAYLOAD_REQ:
                LOG_INF("Received Payload request");
                evt.type = DFU_EVENT_CHUNK_REQUEST;
                memcpy( (uint8_t*) &evt.event.chunk_request, &msg.frame_addr[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], sizeof(bm_dfu_event_chunk_request_t));
                if (k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT))
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_PAYLOAD:
                evt.type = DFU_EVENT_IMAGE_CHUNK;
                evt.event.img_chunk.payload_length = msg.frame_length;
                evt.event.img_chunk.payload_buf = msg.frame_addr;
                if (k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT))
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_END:
                LOG_INF("Received DFU end");
                evt.type = DFU_EVENT_UPDATE_END;
                memcpy( (uint8_t*) &evt.event.update_end, &msg.frame_addr[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], sizeof(bm_dfu_event_update_end_t));
                if (k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT))
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_ACK:
                LOG_INF("Received ACK");
                evt.type = DFU_EVENT_ACK_RECEIVED;
                memcpy( (uint8_t*) &evt.event.ack_received, &msg.frame_addr[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], sizeof(bm_dfu_event_ack_received_t));
                if (k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT))
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_ABORT:
                LOG_INF("Received Abort");
                evt.type = DFU_EVENT_ABORT;
                memcpy( (uint8_t*) &evt.event.abort, &msg.frame_addr[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], sizeof(bm_dfu_event_abort_t));
                if (k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT))
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_HEARTBEAT:
                // LOG_INF("Received Heartbeat");
                evt.type = DFU_EVENT_HEARTBEAT;
                memcpy( (uint8_t*) &evt.event.heartbeat, &msg.frame_addr[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], sizeof(bm_dfu_event_heartbeat_t));
                if (k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT))
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            case BM_DFU_BEGIN_HOST:
                LOG_INF("Received DFU Begin from Desktop");
                evt.type = DFU_EVENT_BEGIN_HOST;
                memcpy( (uint8_t*) &evt.event.begin_host, &msg.frame_addr[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], sizeof(bm_dfu_event_begin_host_t));
                if (k_msgq_put(&_dfu_subsystem_queue, &evt, K_NO_WAIT))
                {
                    LOG_ERR("Message could not be added to Queue");
                }
                break;
            default:
                /* TODO: What do we do here? */ 
                break;
        }
    }
}

/**
 * BM DFU Subsystem Thread (Runs HFSM in an event-driven manner)
 */

/**
 * @brief BM DFU Subsystem Thread (Runs HFSM in an event-driven manner)
 *
 * @note Events are stored in DFU core context and the current state's run function is called
 *
 * @param *arg    Required by Zephyr
 * @return none
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
            LOG_ERR("State Machine Error: terminating");
            break;
        }
    }
}

/**
 * @brief Initialization function for the DFU Core subsystem
 *
 * @note HFSM and Transport service threads are created with Initial State set
 *
 * @param *arg    Required by Zephyr
 * @return none
 */
static int bm_dfu_init( const struct device *arg )
{
    k_tid_t thread_id;
    bm_dfu_event_t evt;
    int retval;

    ARG_UNUSED(arg);

    /* Set initial state */
    smf_set_initial(SMF_CTX(&_dfu_context), &dfu_states[BM_DFU_STATE_INIT]);

    /* Create Transport Service thread to create events for HFSM */
    thread_id = k_thread_create(&_transport_service_dfu_thread_data, _bm_dfu_transport_service_stack,
            K_THREAD_STACK_SIZEOF(_bm_dfu_transport_service_stack),
            (k_thread_entry_t)bm_dfu_transport_service_thread,
            NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);

    if (thread_id == NULL)
    {
        LOG_ERR("DFU Transport Service thread not created");
        return -1;
    }

    /* Thread to run HFSM */
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
            return -1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        LOG_ERR("DFU Subsystem thread not created");
        return -1;
    }
}

/**
 * @brief Get DFU Transport Service Queue
 *
 * @note Used by Transport Subsystems (like BM Serial) to place DFU-specific messages onto the DFU Transport Message Queue
 * 
 * @param none
 * @return k_msgq* Pointer to DFU Subsystem Zephyr Message Queue
 */

struct k_msgq* bm_dfu_get_transport_service_queue(void)
{
    return &_dfu_transport_service_queue;
}

/**
 * @brief Get DFU Subsystem Event Queue
 *
 * @note Used by DFU host and client contexts to put events into the Subsystem Queue
 * 
 * @param none
 * @return k_msgq* Pointer to DFU Subsystem Zephyr Message Queue
 */
struct k_msgq* bm_dfu_get_subsystem_queue(void)
{
    return &_dfu_subsystem_queue;
}

/**
 * @brief Get latest DFU event
 *
 * @note Get the event currently stored in the DFU Core context
 * 
 * @param none
 * @return bm_dfu_event_t Latest DFU Event enum
 */
bm_dfu_event_t bm_dfu_get_current_event(void)
{
    return _dfu_context.current_event;
}

/**
 * @brief Set DFU State
 *
 * @note Set the state of the HFSM in the DFU Core
 * 
 * @param state  Specific DFU State value
 * @return none
 */
void bm_dfu_set_state(uint8_t state)
{
    smf_set_state(SMF_CTX(&_dfu_context), &dfu_states[state]);
}


/**
 * @brief Set DFU Core Error
 *
 * @note Set the error of the DFU context which will be used by the Error State logic
 * 
 * @param error  Specific DFU Error value
 * @return none
 */
void bm_dfu_set_error(uint8_t error)
{
    _dfu_context.error = error;
}


/**
 * @brief Send Heartbeat to other device
 *
 * @note Put DFU Heartbeat BM serial frame into BM Serial TX Queue
 *
 * @return none
 */
void bm_dfu_send_heartbeat(void)
{
    bm_frame_header_t frm_hdr;
    bm_frame_t *dfu_heartbeat_frm;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = sizeof(bm_dfu_frame_header_t);

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
    tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_HEARTBEAT;
    
    dfu_heartbeat_frm = (bm_frame_t *)tx_buf;
    if (bm_serial_frm_put(dfu_heartbeat_frm, BM_END_DEVICE))
    {
        LOG_ERR("Chunk Request not sent");
    }
}

/**
 * @brief Send ACK/NACK to Host
 *
 * @note Stuff ACK bm_frame with success and err_code and put into BM Serial TX Queue
 *
 * @param dev_type      Recipient Device (Desktop or End Device)
 * @param success       1 for ACK, 0 for NACK
 * @param err_code      Error Code enum, read by Host on NACK
 * @return none
 */
void bm_dfu_send_ack(uint8_t dev_type, uint8_t success, uint8_t err_code)
{
    bm_frame_header_t frm_hdr;
    bm_frame_t *ack_frm;
    bm_dfu_event_ack_received_t ack_evt;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_event_ack_received_t) + sizeof(bm_dfu_frame_header_t)];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = sizeof(bm_dfu_event_ack_received_t) + sizeof(bm_dfu_frame_header_t);

    /* Stuff ACK Event */
    ack_evt.success = success;
    ack_evt.err_code = err_code;

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
    tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_ACK;
    memcpy(&tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], (uint8_t *) &ack_evt, sizeof(ack_evt));
    
    ack_frm = (bm_frame_t *)tx_buf;
    if (bm_serial_frm_put(ack_frm, dev_type))
    {
        LOG_ERR("ACK not sent");
    }
}

/**
 * @brief Send Chunk Request
 * 
 * @note Stuff Chunk Request bm_frame with chunk number and put into BM Serial TX Queue
 * 
 * @param dev_type      Recipient Device (Desktop or End Device)
 * @param chunk_num     Image Chunk number requested
 * @return none
 */
void bm_dfu_req_next_chunk(uint8_t dev_type, uint16_t chunk_num)
{
    bm_frame_header_t frm_hdr;
    bm_frame_t *chunk_req_frm;
    bm_dfu_event_chunk_request_t chunk_req_evt;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_event_chunk_request_t) + sizeof(bm_dfu_frame_header_t)];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = sizeof(bm_dfu_event_chunk_request_t) + sizeof(bm_dfu_frame_header_t);

    /* Stuff Chunk Request Event */
    chunk_req_evt.seq_num = chunk_num;

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
    tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_PAYLOAD_REQ;
    memcpy(&tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], (uint8_t *) &chunk_req_evt, sizeof(chunk_req_evt));
    
    chunk_req_frm = (bm_frame_t *)tx_buf;
    if (bm_serial_frm_put(chunk_req_frm, dev_type))
    {
        LOG_ERR("Chunk Request not sent");
    }
}

/**
 * @brief Send DFU END
 *
 * @note Stuff DFU END bm_frame with success and err_code and put into BM Serial TX Queue
 *
 * @param dev_type      Recipient Device (Desktop or End Device)
 * @param success       1 for Successful Update, 0 for Unsuccessful
 * @param err_code      Error Code enum, read by Host on Unsuccessful update
 * @return none
 */
void bm_dfu_update_end(uint8_t dev_type, uint8_t success, uint8_t err_code)
{
    bm_frame_header_t frm_hdr;
    bm_frame_t *update_end_frm;
    bm_dfu_event_update_end_t update_end_evt;

    uint8_t tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_event_update_end_t) + sizeof(bm_dfu_frame_header_t)];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = sizeof(bm_dfu_event_update_end_t) + sizeof(bm_dfu_frame_header_t);

    /* Stuff Update End Event */
    update_end_evt.success = success;
    update_end_evt.err_code = err_code;

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
    tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_END;
    memcpy(&tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], (uint8_t *) &update_end_evt, sizeof(update_end_evt));

    update_end_frm = (bm_frame_t *)tx_buf;
    if (bm_serial_frm_put(update_end_frm, dev_type))
    {
        LOG_ERR("DFU End not sent");
        return;
    }
}

SYS_INIT( bm_dfu_init, POST_KERNEL, 1 );