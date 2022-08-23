#include <init.h>
#include <logging/log.h>
#include <storage/flash_map.h>

#include <zephyr/bm/bm_dfu_serial.h>
#include <zephyr/bm/bm_dfu.h>
#include <zephyr/bm/bm_dfu_host.h>
#include <zephyr/bm/bm_common.h>


LOG_MODULE_REGISTER(bm_dfu_host, CONFIG_BM_LOG_LEVEL);

static dfu_host_ctx_t _host_context;
static int sock;

static struct sockaddr_in6 out_addr6;
static struct sockaddr* out_addr;

static struct sockaddr_in6 client_addr6;

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

    LOG_ERR("Ack Timeout");
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

    LOG_ERR("Heartbeat Timeout");
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
    int retval;
    bm_dfu_event_update_request_t update_req_evt;
    uint8_t tx_buf[sizeof(bm_dfu_event_update_request_t) + sizeof(bm_dfu_frame_header_t)];

    LOG_DBG("Sending Update to Client");
   
    struct in6_addr* addr = net_if_ipv6_get_global_addr(NET_ADDR_PREFERRED, NULL);

    /* Stuff Update Request Event */
    update_req_evt.img_info = _host_context.img_info;
    memcpy(update_req_evt.src_addr, addr->s6_addr, sizeof(addr->s6_addr));
    memcpy(update_req_evt.dst_addr, client_addr6.sin6_addr.s6_addr, sizeof(client_addr6.sin6_addr.s6_addr));

    tx_buf[0] = BM_DFU_START;
    memcpy(&tx_buf[sizeof(bm_dfu_frame_header_t)], (uint8_t *) &update_req_evt, sizeof(update_req_evt));
    
    retval = zsock_sendto(sock, &tx_buf, sizeof(tx_buf), 0, out_addr, sizeof(struct sockaddr_in6));
	if (retval < 0)
    {
		LOG_ERR("Failed to send, errno %d", errno);
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
    int retval;
    uint8_t tx_buf[_host_context.chunk_length + (2 * sizeof(client_addr6.sin6_addr.s6_addr)) + sizeof(uint16_t) + sizeof(bm_dfu_frame_header_t)];

    struct in6_addr* addr = net_if_ipv6_get_global_addr(NET_ADDR_PREFERRED, NULL);

    tx_buf[0] = BM_DFU_PAYLOAD;
    memcpy(&tx_buf[sizeof(bm_dfu_frame_header_t)], addr->s6_addr, sizeof(addr->s6_addr));
    memcpy(&tx_buf[sizeof(bm_dfu_frame_header_t) + sizeof(client_addr6.sin6_addr.s6_addr)], client_addr6.sin6_addr.s6_addr, sizeof(client_addr6.sin6_addr.s6_addr));
    tx_buf[sizeof(bm_dfu_frame_header_t) + (2 * sizeof(client_addr6.sin6_addr.s6_addr))] = _host_context.chunk_length;
    memcpy(&tx_buf[sizeof(bm_dfu_frame_header_t) + (2 * sizeof(client_addr6.sin6_addr.s6_addr)) + sizeof(uint16_t)], _host_context.chunk_buf , _host_context.chunk_length);
    
    retval = zsock_sendto(sock, &tx_buf, sizeof(tx_buf), 0, out_addr, sizeof(struct sockaddr_in6));
	if (retval < 0)
    {
		LOG_ERR("Failed to send, errno %d", errno);
	}
}

/**
 * @brief Initialization function for the DFU Host subsystem
 *
 * @note Subsystem timeout timers are created and Event Queue is grabbed from DFU core
 *
 * @param sock    Socket from DFU Host
 * @return none
 */
int bm_dfu_host_init( int _sock)
{
    /* Socket from DFU Core */
    sock = _sock;

    out_addr6.sin6_family = AF_INET6;
    out_addr6.sin6_port = htons(BM_DFU_SOCKET_PORT);
    zsock_inet_pton(AF_INET6, "ff03::1", &out_addr6.sin6_addr);
    out_addr = (struct sockaddr *)&out_addr6;

    /* Get DFU Subsystem Queue */
    _host_context.dfu_subystem_queue = bm_dfu_get_subsystem_queue();

    /* Get DFU Sem */
    _host_context.dfu_sem = bm_dfu_serial_get_sem();

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
    bm_dfu_event_t curr_evt = bm_dfu_get_current_event();
    _host_context.img_info = curr_evt.event.begin_host.img_info;

    client_addr6.sin6_family = AF_INET6;
    client_addr6.sin6_port = htons(BM_DFU_SOCKET_PORT);

    printk("IPv6 Address: %s\n", curr_evt.event.begin_host.dst_ipv6_addr);

    zsock_inet_pton(AF_INET6, curr_evt.event.begin_host.dst_ipv6_addr, &client_addr6.sin6_addr.s6_addr);

    printk("IPv6 Address: %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x\n", client_addr6.sin6_addr.s6_addr[0], 
                                                                                                         client_addr6.sin6_addr.s6_addr[1], 
                                                                                                         client_addr6.sin6_addr.s6_addr[2], 
                                                                                                         client_addr6.sin6_addr.s6_addr[3],
                                                                                                         client_addr6.sin6_addr.s6_addr[4], 
                                                                                                         client_addr6.sin6_addr.s6_addr[5], 
                                                                                                         client_addr6.sin6_addr.s6_addr[6], 
                                                                                                         client_addr6.sin6_addr.s6_addr[7], 
                                                                                                         client_addr6.sin6_addr.s6_addr[8],
                                                                                                         client_addr6.sin6_addr.s6_addr[9], 
                                                                                                         client_addr6.sin6_addr.s6_addr[10], 
                                                                                                         client_addr6.sin6_addr.s6_addr[11], 
                                                                                                         client_addr6.sin6_addr.s6_addr[12], 
                                                                                                         client_addr6.sin6_addr.s6_addr[13],
                                                                                                         client_addr6.sin6_addr.s6_addr[14], 
                                                                                                         client_addr6.sin6_addr.s6_addr[15]);

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

        bm_dfu_send_ack(BM_DESKTOP, NULL, curr_evt.event.ack_received.success, curr_evt.event.ack_received.err_code);

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
            bm_dfu_send_ack(BM_DESKTOP, NULL, 0, BM_DFU_ERR_TIMEOUT);
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
    bm_dfu_event_t curr_evt = bm_dfu_get_current_event();
    if (curr_evt.type == DFU_EVENT_CHUNK_REQUEST)
    {
        k_timer_stop(&_host_context.heartbeat_timer);

        /* Request Next Chunk */
        bm_dfu_req_next_chunk(BM_DESKTOP, NULL, curr_evt.event.chunk_request.seq_num);

        /* Send Heartbeat to Client 
            TODO: Make this a periodic heartbeat in case it takes a while to grab chunk from external host
        */
        bm_dfu_send_heartbeat(&client_addr6);
    }
    else if (curr_evt.type == DFU_EVENT_IMAGE_CHUNK)
    {
        k_sem_take(_host_context.dfu_sem , K_FOREVER);
        _host_context.chunk_length = curr_evt.event.img_chunk.payload_length - (sizeof(bristlemouth_frame_header_t) + sizeof(bm_dfu_frame_header_t));
        memcpy(_host_context.chunk_buf, &curr_evt.event.img_chunk.payload_buf[sizeof(bristlemouth_frame_header_t) + sizeof(bm_dfu_frame_header_t)], _host_context.chunk_length);
        k_sem_give(_host_context.dfu_sem);

        bm_dfu_host_send_chunk();
        k_timer_start((struct k_timer*) &_host_context.heartbeat_timer, K_USEC(BM_DFU_HOST_HEARTBEAT_TIMEOUT), K_NO_WAIT);
    }
    else if (curr_evt.type == DFU_EVENT_UPDATE_END)
    {
        k_timer_stop(&_host_context.heartbeat_timer);

        bm_dfu_update_end(BM_DESKTOP, NULL, curr_evt.event.update_end.success, curr_evt.event.update_end.err_code);

        if (curr_evt.event.update_end.success)
        {
            LOG_INF("Successfully updated Client");
        }
        else
        {
            LOG_ERR("Client Update Failed");
        }

        bm_dfu_set_state(BM_DFU_STATE_IDLE);
    }
    else if (curr_evt.type == DFU_EVENT_HEARTBEAT)
    {
        k_timer_stop(&_host_context.heartbeat_timer);
        k_timer_start((struct k_timer*) &_host_context.heartbeat_timer, K_USEC(BM_DFU_HOST_HEARTBEAT_TIMEOUT), K_NO_WAIT);
    }
    else if (curr_evt.type == DFU_EVENT_HEARTBEAT_TIMEOUT)
    {
        bm_dfu_update_end(BM_DESKTOP, NULL, 0, BM_DFU_ERR_TIMEOUT);
        bm_dfu_set_error(BM_DFU_ERR_TIMEOUT);
        bm_dfu_set_state(BM_DFU_STATE_ERROR);
    }
    else if (curr_evt.type == DFU_EVENT_ABORT)
    {
        bm_dfu_update_end(BM_DESKTOP, NULL, 0, BM_DFU_ERR_ABORTED);
        bm_dfu_set_state(BM_DFU_STATE_IDLE);
    }
}