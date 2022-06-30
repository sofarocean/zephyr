#include <init.h>
#include <logging/log.h>
#include <storage/flash_map.h>
#include <bootutil/bootutil_public.h>
#include <dfu/mcuboot.h>
#include <sys/reboot.h>

#include <drivers/bm/bm_serial.h>
#include <drivers/bm/bm_dfu.h>
#include <drivers/bm/bm_dfu_client.h>
#include <drivers/bm/bm_common.h>

LOG_MODULE_REGISTER(bm_dfu_client, CONFIG_BM_LOG_LEVEL);

static dfu_client_ctx_t _client_context;

static void bm_dfu_client_req_next_chunk(void)
{
    //LOG_INF("Requesting Next Chunk");
    bm_frame_header_t frm_hdr;
    bm_frame_t *chunk_req_frm;
    bm_dfu_event_chunk_request_t chunk_req_evt;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_event_chunk_request_t) + sizeof(bm_dfu_frame_header_t)];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = sizeof(bm_dfu_event_chunk_request_t) + sizeof(bm_dfu_frame_header_t);

    /* Stuff Chunk Request Event */
    chunk_req_evt.seq_num = _client_context.current_chunk;

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_PAYLOAD_REQ;
    memcpy(&tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], (uint8_t *) &chunk_req_evt, sizeof(chunk_req_evt));
    
    chunk_req_frm = (bm_frame_t *)tx_buf;
    if (bm_serial_frm_put(chunk_req_frm))
    {
        LOG_ERR("Chunk Request not sent");
    }
}

static void bm_dfu_client_send_ack(uint8_t success, uint8_t err_code)
{
    LOG_INF("Sending ACK");
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
    if (bm_serial_frm_put(ack_frm))
    {
        LOG_ERR("ACK not sent");
    }
}

static void bm_dfu_client_abort(void)
{
    LOG_INF("Sending ABORT");
    bm_frame_header_t frm_hdr;
    bm_frame_t *abort_frm;
    uint8_t tx_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)];

    /* Stuff BM Frame Header*/
    frm_hdr.version = BM_V0;
    frm_hdr.payload_type = BM_DFU;
    frm_hdr.payload_length = sizeof(bm_dfu_frame_header_t);

    memcpy(tx_buf, &frm_hdr, sizeof(bm_frame_header_t));
	tx_buf[sizeof(bm_frame_header_t)] = BM_DFU_ABORT;

    abort_frm = (bm_frame_t *)tx_buf;
    if (bm_serial_frm_put(abort_frm))
    {
        LOG_ERR("ABORT not sent");
    }
}

static void bm_dfu_client_update_end(uint8_t success, uint8_t err_code)
{
    LOG_INF("Sending Update End");
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
    if (bm_serial_frm_put(update_end_frm))
    {
        LOG_ERR("DFU End not sent");
        return;
    }
}

static void chunk_timer_handler(struct k_timer *tmr)
{
    LOG_INF("Chunk Timer Timeout");
    bm_dfu_event_t evt;

    evt.type = DFU_EVENT_CHUNK_TIMEOUT;
    if(k_msgq_put(_client_context.dfu_subystem_queue, &evt, K_NO_WAIT))
    {
        LOG_ERR("Message could not be added to Queue");
    }
}

static int bm_dfu_process_payload(uint16_t man_decode_len, uint8_t * man_decode_buf)
{
    int retval = 0;

    /* Account for BM Frame Header size and BM_DFU payload type*/
    uint16_t _dfu_frame_len = man_decode_len - sizeof(bm_frame_header_t) - sizeof(bm_dfu_frame_header_t);

    if ( BM_IMG_PAGE_LENGTH > (_dfu_frame_len + _client_context.img_page_byte_counter))
    {
        memcpy(&_client_context.img_page_buf[_client_context.img_page_byte_counter], \
                                &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], _dfu_frame_len);
        _client_context.img_page_byte_counter += _dfu_frame_len;

        if (_client_context.img_page_byte_counter == BM_IMG_PAGE_LENGTH)
        {
            _client_context.img_page_byte_counter = 0;

            /* Perform page write and increment flash byte counter */
            retval = flash_area_write(_client_context.fa, _client_context.img_flash_offset, _client_context.img_page_buf, BM_IMG_PAGE_LENGTH);
            if (retval)
            {
                LOG_ERR("Unable to write DFU frame to Flash");
                goto out;
            }
            else
            {
                _client_context.img_flash_offset += BM_IMG_PAGE_LENGTH;
            }
        }
    }
    else
    {
        uint16_t _remaining_page_length = BM_IMG_PAGE_LENGTH - _client_context.img_page_byte_counter;
        memcpy(&_client_context.img_page_buf[_client_context.img_page_byte_counter], \
                                &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t)], _remaining_page_length);
        _client_context.img_page_byte_counter += _remaining_page_length;
        
        if (_client_context.img_page_byte_counter == BM_IMG_PAGE_LENGTH)
        {
            _client_context.img_page_byte_counter = 0;

            /* Perform page write and increment flash byte counter */
            retval = flash_area_write(_client_context.fa, _client_context.img_flash_offset, _client_context.img_page_buf, BM_IMG_PAGE_LENGTH);
            if (retval)
            {
                LOG_ERR("Unable to write DFU frame to Flash");
                goto out;
            }
            else
            {
                _client_context.img_flash_offset += BM_IMG_PAGE_LENGTH;
            }
        }
        
        /* Memcpy the remaining bytes to next page */
        memcpy(&_client_context.img_page_buf[_client_context.img_page_byte_counter], \
                                &man_decode_buf[sizeof(bm_frame_header_t) + sizeof(bm_dfu_frame_header_t) + _remaining_page_length], \
                                (_dfu_frame_len - _remaining_page_length) );
        _client_context.img_page_byte_counter += (_dfu_frame_len - _remaining_page_length);
    }
out:
    return retval;
}

static int bm_dfu_process_end(void)
{
    int retval = 0;

    /* If there are any dirty bytes, write to flash */
    if (_client_context.img_page_byte_counter != 0)
    {
        uint8_t _dfu_pad_counter = 0;

        /* STM32L4 needs flash writes to be 8-byte aligned */
        while ( (_client_context.img_page_byte_counter + _dfu_pad_counter) % 8 )
        {
            _dfu_pad_counter++;
            _client_context.img_page_buf[_client_context.img_page_byte_counter + _dfu_pad_counter] = 0;
        }

        /* Perform page write and increment flash byte counter */
        retval = flash_area_write(_client_context.fa, _client_context.img_flash_offset, _client_context.img_page_buf, (_client_context.img_page_byte_counter + _dfu_pad_counter));
        if (retval)
        {
            LOG_ERR("Unable to write DFU frame to Flash");
            goto out;
        }
        else
        {
            _client_context.img_flash_offset += _client_context.img_page_byte_counter;
        }
    }
out:
    return retval;
}

static int bm_dfu_client_init( const struct device *arg )
{
    ARG_UNUSED(arg);

    /* Get DFU Sem */
    _client_context.dfu_sem = bm_serial_get_dfu_sem();

    /* Get DFU Subsystem Queue */
    _client_context.dfu_subystem_queue = bm_dfu_get_subsystem_queue();

    /* Initialize Chunk Timer */
    k_timer_init(( struct k_timer* ) &_client_context.chunk_timer, chunk_timer_handler, NULL);

    return 0; 
}

struct dfu_client_ctx_t* bm_dfu_client_get_context(void)
{
    return &_client_context;
}

void bm_dfu_client_process_request(void)
{
    uint32_t image_size;
    uint16_t chunk_size;
    uint8_t minor_version;
    uint8_t major_version;

    bm_dfu_event_t curr_evt = bm_dfu_get_current_event();

    image_size = curr_evt.event.update_request.img_info.image_size;
    chunk_size = curr_evt.event.update_request.img_info.chunk_size;
    minor_version = curr_evt.event.update_request.img_info.minor_ver;
    major_version = curr_evt.event.update_request.img_info.major_ver;

    /* TODO: Is there a way to grab the partition size from the .dts file? */
    if (image_size <= 0x67000)
    {
        /* TODO: Need to check min/major version numbers */
        if (1)
        {
            _client_context.image_size = image_size;
            if (image_size % chunk_size)
            {
                _client_context.num_chunks = ( image_size / chunk_size ) + 1;
            }
            else
            {
                _client_context.num_chunks = ( image_size / chunk_size );
            }
            _client_context.crc16 = curr_evt.event.update_request.img_info.crc16;

             /* Open the secondary image slot */
            if (flash_area_open(FLASH_AREA_ID(image_1), &_client_context.fa) != 0)
            {
                LOG_ERR("Flash driver was not found!\n");
                bm_dfu_client_send_ack(0, BM_DFU_ERR_FLASH_ACCESS);
            }
            else
            {
                /* Erase memory in secondary image slot */
                if (boot_erase_img_bank(FLASH_AREA_ID(image_1)) != 0)
                {
                    LOG_ERR("Unable to erase Secondary Image slot");
                    bm_dfu_client_send_ack(0, BM_DFU_ERR_FLASH_ACCESS);
                }
                else
                {
                    LOG_INF("Erase complete");
                    bm_dfu_client_send_ack(1, BM_DFU_ERR_NONE);
                    //k_sleep(K_USEC(100));
                    bm_dfu_set_state(BM_DFU_STATE_CLIENT_RECEIVING);
                }
            }
        }
        else
        {
            bm_dfu_client_send_ack(0, BM_DFU_ERR_SAME_VER);
        }
    }
    else
    {
        bm_dfu_client_send_ack(0, BM_DFU_ERR_TOO_LARGE);
    }
}


void s_client_entry(void *o)
{
    LOG_INF("Client State entry");
}

void s_client_exit(void *o)
{
    /* TODO: What do we do here? */
}


void s_client_receiving_entry(void *o)
{
    LOG_INF("Client Receiving State entry");
    /* Start from Chunk #0 */
    _client_context.current_chunk = 0;
    _client_context.chunk_retry_num = 0;

    /* Request Next Chunk */
    bm_dfu_client_req_next_chunk();

    /* Kickoff Chunk timeout */
    k_timer_start((struct k_timer*) &_client_context.chunk_timer, K_USEC(BM_DFU_CLIENT_CHUNK_TIMEOUT), K_NO_WAIT);
}

void s_client_receiving_run(void *o)
{
    bm_dfu_event_t curr_evt = bm_dfu_get_current_event();
    if (curr_evt.type == DFU_EVENT_IMAGE_CHUNK)
    {
        /* Stop Chunk Timer */
        k_timer_stop(&_client_context.chunk_timer);

        //k_sem_take(_client_context.dfu_sem , K_FOREVER);
        _client_context.chunk_length = curr_evt.event.img_chunk.payload_length;
        memcpy(_client_context.chunk_buf, curr_evt.event.img_chunk.payload_buf, _client_context.chunk_length);
        //k_sem_give(_client_context.dfu_sem);

        /* Process the frame */
        if (bm_dfu_process_payload(_client_context.chunk_length, _client_context.chunk_buf))
        {
            bm_dfu_set_state(BM_DFU_STATE_ERROR);
        }

        /* Request Next Chunk */
        _client_context.current_chunk++;
        _client_context.chunk_retry_num = 0;

        if (_client_context.current_chunk < _client_context.num_chunks )
        {
            bm_dfu_client_req_next_chunk();
        }
        else
        {
            /* Process the frame */
            if (bm_dfu_process_end())
            {
                bm_dfu_set_state(BM_DFU_STATE_ERROR);
            }
            else
            {
                bm_dfu_set_state(BM_DFU_STATE_CLIENT_VALIDATING);
            }
        }
    }
    else if (curr_evt.type == DFU_EVENT_CHUNK_TIMEOUT)
    {
        _client_context.chunk_retry_num++;

        /* Try requesting chunk until max retries is reached */
        if (_client_context.chunk_retry_num >= BM_DFU_MAX_CHUNK_RETRIES)
        {
            bm_dfu_client_abort();
            bm_dfu_set_state(BM_DFU_STATE_IDLE);
        }
        else
        {
            bm_dfu_client_req_next_chunk();
            k_timer_start((struct k_timer*) &_client_context.chunk_timer, K_USEC(BM_DFU_CLIENT_CHUNK_TIMEOUT), K_NO_WAIT);
        }
    }
}

void s_client_validating_entry(void *o)
{
    LOG_INF("Client Validating Entry");
    /* Verify image length */
    if (_client_context.image_size != _client_context.img_flash_offset)
    {
        bm_dfu_client_update_end(0, BM_DFU_ERR_MISMATCH_LEN);
        bm_dfu_set_state(BM_DFU_STATE_IDLE);
    }
    else
    {
        /* TODO: Verify CRC. If ok, then move to Activating state*/
        if (1)
        {
            bm_dfu_client_update_end(1, BM_DFU_ERR_NONE);
            bm_dfu_set_state(BM_DFU_STATE_CLIENT_ACTIVATING);
        }
        else
        {
            bm_dfu_client_update_end(0, BM_DFU_ERR_BAD_CRC);
            bm_dfu_set_state(BM_DFU_STATE_IDLE);
        }
    }
}

void s_client_activating_entry(void *o)
{
    LOG_INF("Client Activating Entry");
    /* Set as temporary switch. New application must confirm or else MCUBoot will
    switch back to old image */
    boot_set_pending(0);
    //sys_reboot(0);
}

SYS_INIT( bm_dfu_client_init, POST_KERNEL, 2 );