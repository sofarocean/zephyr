#include <init.h>
#include <logging/log.h>
#include <storage/flash_map.h>
#include <bootutil/bootutil_public.h>
#include <dfu/mcuboot.h>
#include <sys/reboot.h>

#include <drivers/bm/bm_serial.h>
#include <drivers/bm/bm_dfu.h>
#include <drivers/bm/bm_dfu_host.h>
#include <drivers/bm/bm_common.h>

LOG_MODULE_REGISTER(bm_dfu_host, CONFIG_BM_LOG_LEVEL);

static dfu_host_ctx_t _host_context;

/**
 * @brief ACK Timer Handler function
 *
 * @note Puts ACK Timeout event into DFU Subsystem event queue
 *
 * @param *tmr    Pointer to Zephyr Timer struct
 * @return none
 */
static void ack_timer_handler(struct k_timer *tmr)
{
    bm_dfu_event_t evt;

    evt.type = DFU_EVENT_ACK_TIMEOUT;
    if (k_msgq_put(_host_context.dfu_subystem_queue, &evt, K_NO_WAIT))
    {
        LOG_ERR("Message could not be added to Queue");
    }
}

/**
 * @brief Heartbeat Timer Handler function
 *
 * @note Puts Heartbeat Timeout event into DFU Subsystem event queue
 *
 * @param *tmr    Pointer to Zephyr Timer struct
 * @return none
 */
static void heartbeat_timer_handler(struct k_timer *tmr)
{
    bm_dfu_event_t evt;

    evt.type = DFU_EVENT_HEARTBEAT_TIMEOUT;
    if(k_msgq_put(_host_context.dfu_subystem_queue, &evt, K_NO_WAIT))
    {
        LOG_ERR("Message could not be added to Queue");
    }
}

/**
 * @brief Send Request Update to Client
 *
 * @note Stuff Update Request bm_frame with image info and put into BM Serial TX Queue
 *
 * @return none
 */
static void bm_dfu_host_req_update(void)
{
    bm_frame_header_t frm_hdr;
    bm_frame_t *dfu_req_frm;
    bm_dfu_event_update_request_t update_req_evt;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_event_update_request_t) + sizeof(bm_dfu_frame_header_t)];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = sizeof(bm_dfu_event_update_request_t) + sizeof(bm_dfu_frame_header_t);

    /* Stuff Update Request Event */
    update_req_evt.img_info = _host_context.img_info;

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_START;
    memcpy(&tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], (uint8_t *) &update_req_evt, sizeof(update_req_evt));
    
    dfu_req_frm = (bm_frame_t *)tx_buf;
    if (bm_serial_frm_put(dfu_req_frm))
    {
        LOG_ERR("Chunk Request not sent");
    }
}

/**
 * @brief Send Chunk to Client
 *
 * @note Stuff bm_frame with image chunk and put into BM Serial TX Queue
 *
 * @return none
 */
static void bm_dfu_host_send_chunk(void)
{
    bm_frame_header_t frm_hdr;
    bm_frame_t *dfu_send_chunk_frm;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + _host_context.chunk_length + sizeof(bm_dfu_frame_header_t)];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = _host_context.chunk_length + sizeof(bm_dfu_frame_header_t);

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_PAYLOAD;
    memcpy(&tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], _host_context.chunk_buf , _host_context.chunk_length);
    
    dfu_send_chunk_frm = (bm_frame_t *)tx_buf;
    if (bm_serial_frm_put(dfu_send_chunk_frm))
    {
        LOG_ERR("Chunk Request not sent");
    }
}

/**
 * @brief Initialization function for the DFU Host subsystem
 *
 * @note Subsystem timeout timers are created and Event Queue is grabbed from DFU core
 *
 * @param *arg    Required by Zephyr
 * @return none
 */
static int bm_dfu_host_init( const struct device *arg )
{
    ARG_UNUSED(arg);

    /* Get DFU Subsystem Queue */
    _host_context.dfu_subystem_queue = bm_dfu_get_subsystem_queue();

    /* Initialize ACK and Heartbeat Timer */
    k_timer_init(( struct k_timer* ) &_host_context.ack_timer, ack_timer_handler, NULL);
    k_timer_init(( struct k_timer* ) &_host_context.heartbeat_timer, heartbeat_timer_handler, NULL);

    return 0; 
}

/**
 * @brief Entry Function for the High-level HOST State
 *
 * @note Currently empty. Will be run before the child enter state function. TODO: How should we "restart" an update if the 
 *       host device jumps from a host to client state?
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
void s_host_entry(void *o)
{
    /* TODO: What should we do when we enter the Host FSM */
}

/**
 * @brief Exit Function for the High-level HOST State
 *
 * @note Currently empty. Will be run after the child exit state function. TODO: How should we "pause" an update if the 
 *       host device jumps from a host to client state?
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
void s_host_exit(void *o)
{
    /* TODO: What should we do when we exit the Host FSM */
}

/**
 * @brief Entry Function for the Request Update State
 *
 * @note The Host sends an update request to the client and starts the ACK timeout timer
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
void s_host_req_update_entry(void *o)
{
    _host_context.ack_retry_num = 0;

    /* Request Client Firmware Update */
    bm_dfu_host_req_update();

    /* Kickoff ACK timeout */
    k_timer_start((struct k_timer*) &_host_context.ack_timer, K_USEC(BM_DFU_HOST_ACK_TIMEOUT), K_NO_WAIT);
}

/**
 * @brief Run Function for the Request Update State
 *
 * @note The state is waiting on an ACK from the client to begin the update. Returns to idle state on timeout
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
void s_host_req_update_run(void *o)
{
    bm_dfu_event_t curr_evt = bm_dfu_get_current_event();
    if (curr_evt.type == DFU_EVENT_ACK_RECEIVED)
    {
        /* Stop ACK Timer */
        k_timer_stop(&_host_context.ack_timer);

        if (curr_evt.event.ack_received.success)
        {
            bm_dfu_set_state(BM_DFU_STATE_HOST_UPDATE);
        }
        else
        {
            bm_dfu_set_error(curr_evt.event.ack_received.err_code);
            bm_dfu_set_state(BM_DFU_STATE_ERROR);
        }
    }
    else if (curr_evt.type == DFU_EVENT_ACK_TIMEOUT)
    {
        _host_context.ack_retry_num++;

        /* Wait for ack until max retries is reached */
        if (_host_context.ack_retry_num >= BM_DFU_MAX_CHUNK_RETRIES)
        {
            bm_dfu_set_error(BM_DFU_ERR_TIMEOUT);
            bm_dfu_set_state(BM_DFU_STATE_ERROR);
        }
        else
        {
            bm_dfu_host_req_update();
            k_timer_start((struct k_timer*) &_host_context.ack_timer, K_USEC(BM_DFU_HOST_ACK_TIMEOUT), K_NO_WAIT);
        }
    }
}

/**
 * @brief Entry Function for the Update State
 *
 * @note The Host starts the Heartbeat timeout timer
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
void s_host_update_entry(void *o)
{
    k_timer_start((struct k_timer*) &_host_context.heartbeat_timer, K_USEC(BM_DFU_HOST_HEARTBEAT_TIMEOUT), K_NO_WAIT);
}

/**
 * @brief Run Function for the Update State
 *
 * @note Host state that sends chunks of image to Client. Exits on heartbeat timeout or end message received from client
 *
 * @param *o    Required by zephyr smf library for state functions
 * @return none
 */
void s_host_update_run(void *o)
{
    int retval;
    bm_dfu_event_t curr_evt = bm_dfu_get_current_event();
    if (curr_evt.type == DFU_EVENT_CHUNK_REQUEST)
    {
        k_timer_stop(&_host_context.heartbeat_timer);
        retval = _host_context.req_cb(curr_evt.event.chunk_request.seq_num, &_host_context.chunk_length, _host_context.chunk_buf, sizeof(_host_context.chunk_buf));
        if (retval == 0)
        {
            bm_dfu_host_send_chunk();
            k_timer_start((struct k_timer*) &_host_context.heartbeat_timer, K_USEC(BM_DFU_HOST_HEARTBEAT_TIMEOUT), K_NO_WAIT);
        }
        else
        {
            bm_dfu_set_error(BM_DFU_ERR_IMG_CHUNK_ACCESS);
            bm_dfu_set_state(BM_DFU_STATE_ERROR);
        }
    }
    else if (curr_evt.type == DFU_EVENT_UPDATE_END)
    {
        k_timer_stop(&_host_context.heartbeat_timer);
        if (curr_evt.event.update_end.success)
        {
            LOG_INF("Successfully updated Client");
        }
        else
        {
            LOG_ERR("Client Update Failed");
        }

        /* We haven't heard back from the Client. Let's go back to Idle State */
        bm_dfu_set_state(BM_DFU_STATE_IDLE);
    }
    else if (curr_evt.type == DFU_EVENT_HEARTBEAT)
    {
        k_timer_stop(&_host_context.heartbeat_timer);
        k_timer_start((struct k_timer*) &_host_context.heartbeat_timer, K_USEC(BM_DFU_HOST_HEARTBEAT_TIMEOUT), K_NO_WAIT);
    }
    else if (curr_evt.type == DFU_EVENT_HEARTBEAT_TIMEOUT)
    {
        bm_dfu_set_error(BM_DFU_ERR_TIMEOUT);
        bm_dfu_set_state(BM_DFU_STATE_ERROR);
    }
}

/**
 * @brief Kicks off the Update Process from the Host side
 *
 * @note This will place the "BEGIN_UPDATE" event on the queue
 *
 * @param img_info  Struct of relevant info (size, CRC, etc)
 * @param req_cb    Callback function for grabbing next image chunk
 * @return none
 */
void bm_dfu_host_start_update(bm_dfu_img_info_t *img_info, bm_dfu_chunk_req_cb req_cb)
{
    bm_dfu_event_t evt;
    _host_context.img_info = *img_info;
    _host_context.req_cb = req_cb;

    evt.type = DFU_EVENT_BEGIN_UPDATE;
    if (k_msgq_put(_host_context.dfu_subystem_queue, &evt, K_NO_WAIT))
    {
        LOG_ERR("Message could not be added to Queue");
    }
}

SYS_INIT( bm_dfu_host_init, POST_KERNEL, 2 );