/** @file
 * @brief Pipe UART driver
 *
 * A pipe UART driver allowing application to handle all aspects of received
 * protocol data.
 */

/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(bm_serial, CONFIG_UART_CONSOLE_LOG_LEVEL);

#include <kernel.h>
#include <drivers/uart.h>

#include <sys/printk.h>
#include <drivers/console/bm_serial.h>
#include <sys/crc.h>
#include <cobs.h>

static const struct device* serial_dev[3] = {0};

static struct k_thread tx_thread_data;
static struct k_thread rx_thread_data;

/* Write Buf Pointer for decoding incoming COBS frame */
volatile uint16_t decode_buf_off = 0;

/* Read and Write Pointers to the double buffer */
volatile uint8_t write_buf_idx = 0;
volatile uint8_t read_buf_idx = 1;

/* Buffer to store encoded frame before TX */
uint8_t cobs_encoding_buffer[MAX_ENCODED_BUF_SIZE];

/* Double decoded buffer*/
bm_decoded_t cobs_decoding_buf[2];

/* Buffer for received frame payloads */
uint8_t rx_payload_buf[NUM_BM_FRAMES * MAX_BM_FRAME_SIZE];
uint8_t rx_payload_idx = 0;

/* Buffer for transmitted frame payloads */
uint8_t tx_payload_buf[NUM_BM_FRAMES * MAX_BM_FRAME_SIZE];
uint8_t tx_payload_idx = 0;

/* RX/TX Threads and associated message Queues */
K_THREAD_STACK_DEFINE(bm_tx_stack, TASK_STACK_SIZE);
K_MSGQ_DEFINE(tx_queue, sizeof(bm_msg_t), NUM_BM_FRAMES, 4);
K_THREAD_STACK_DEFINE(bm_rx_stack, TASK_STACK_SIZE);
K_MSGQ_DEFINE(rx_queue, sizeof(bm_msg_t), NUM_BM_FRAMES, 4);

/* Semaphore for ISR and RX_Task to signal availability of Decoded Frame */
K_SEM_DEFINE(decode_sem, 0, 1);

/* Semaphore for ISR and RX_Task to signal processing of frame has finished 
   Begin with initial value of 1 so that ISR can initiate RX*/
K_SEM_DEFINE(processing_sem, 1, 1);

/* COBS Decoding */
static void bm_parse_and_store(uint8_t *rx_byte)
{
	uint8_t byte_in = *rx_byte;	

	static volatile uint16_t len = 0;
	static volatile uint8_t await_alignment = 1;
	static uint8_t input_buffer[ MAX_BM_FRAME_SIZE ];

    // Awaiting complete frame
    if ( await_alignment )
	{
		// If incoming byte is 0, we can now align to the next frame
		if ( byte_in == 0 )
		{
			await_alignment = 0;
			return;
		}
	}
	else
	{
		// Collect bytes until either the next delimiter is reached, or we overflow our len
		if( byte_in == 0 )
		{
			// Frame complete
			//LOG_INF( "Len=%d", len );

			if (len != 0)
			{
				// Decode
				cobs_decode_result_t ret = cobs_decode( cobs_decoding_buf[write_buf_idx].buf, MAX_BM_FRAME_SIZE, input_buffer, len );
				if( ret.status == COBS_DECODE_OK)
				{
					//LOG_INF( "Decoded seq: %d", cobs_decoding_buf[write_buf_idx].buf[ sizeof(bm_frame_header_t)] );

					cobs_decoding_buf[write_buf_idx].length = ret.out_len;

					/* First check if the RX_Task has finished processing read_buf */
					if ( k_sem_take(&processing_sem, K_NO_WAIT))
					{
						LOG_ERR("RX Task has not finished processing data. Overwriting current index of A/B Buffer.");
					}
					else
					{
						/* Flip idx of read and write buffers */
						read_buf_idx = write_buf_idx;
						write_buf_idx = 1 - write_buf_idx;
					}

					/* Notify RX Task that frame is ready to be processed */
					k_sem_give(&decode_sem);
				}
				else
				{
					LOG_ERR( "Failed to decode COBS frame" );
				}
			}
			else
			{
				LOG_ERR("Received an errant 0x00 byte");
			}

			len = 0;
		}
		else
		{
			input_buffer[ len ] = byte_in;
			len++;
		}

		if( len >= MAX_BM_FRAME_SIZE )
		{
			LOG_ERR( "Frame overflow! Len was: %d", len );
			len = 0;

			// Force re-alignment
			await_alignment = 1;
		}
	}

    return;
}

static void bm_serial_read_rx_byte(const struct device *dev)
{
	/* As per the API, the interrupt may be an edge so keep
	 * reading from the FIFO until it's empty.
	 */
	uint8_t byte;
	int got;
	while (1) 
	{
		got = uart_fifo_read(dev, &byte, 1);
		if (got <= 0) 
		{
			break;
		}
		bm_parse_and_store(&byte);
	}
}

/**
 * BM Serial RX Thread
 */
static void bm_serial_rx_thread(void)
{
	uint16_t computed_crc16;
	uint16_t received_crc16;
	uint16_t frame_length;
	int retval;
	LOG_DBG("BM Serial RX thread started");

	while (1)
	{
		// Wait on Producer to finish writing out decoded frame
		k_sem_take(&decode_sem, K_FOREVER);

		frame_length = cobs_decoding_buf[read_buf_idx].length;

		if (frame_length == 0)
		{
			LOG_ERR("Received Frame Length of 0. Skipping");
			k_sem_give(&processing_sem);
			continue;
		}

		//LOG_INF( "Receiving packet of length: %d", frame_length );

		/* Verify CRC16 */
		computed_crc16 = crc16_ccitt(0, cobs_decoding_buf[read_buf_idx].buf, frame_length - sizeof(bm_crc_t));
		received_crc16 = cobs_decoding_buf[read_buf_idx].buf[frame_length - 2];
		received_crc16 |= (cobs_decoding_buf[read_buf_idx].buf[frame_length - 1] << 8);

		if (computed_crc16 != received_crc16)
		{
			LOG_ERR("CRC16 received: %d vs. computed: %d, discarding\n", received_crc16, computed_crc16);
			k_sem_give(&processing_sem);
			continue;
		}

		/* Update frame length with CRC16 removal */
		frame_length -= sizeof(bm_crc_t);

		if(k_msgq_num_free_get(&rx_queue))
		{
			/* Add frame to RX Contiguous Mem */
			memcpy(&rx_payload_buf[rx_payload_idx * MAX_BM_FRAME_SIZE], cobs_decoding_buf[read_buf_idx].buf, frame_length);

			/* Add msg to RX Message Queue (for ieee802154_bm_serial.c RX Task to consume */ 
			bm_msg_t rx_msg = { .frame_addr = &rx_payload_buf[rx_payload_idx * MAX_BM_FRAME_SIZE], .frame_length = frame_length};
			retval = k_msgq_put(&rx_queue, &rx_msg, K_FOREVER);
			if (retval)
			{
				LOG_ERR("Message could not be added to Queue");
			}

			// Update index for storing next RX Payload
			rx_payload_idx++;
			if (rx_payload_idx >= NUM_BM_FRAMES)
			{
				rx_payload_idx = 0;
			}
		}
		else
		{
			LOG_ERR("RX MessageQueue full, dropping message. Get faster!");
		}
		k_sem_give(&processing_sem);
	}
}

/**
 * BM Serial TX Thread
 */
static void bm_serial_tx_thread(void)
{
	LOG_DBG("BM Serial TX thread started");
	bm_msg_t msg;
	uint8_t* frame_addr;
	uint16_t frame_length;
	cobs_encode_result_t enc_retv;
	uint16_t i;
	uint8_t n;

	while (1) 
	{
		k_msgq_get(&tx_queue, &msg, K_FOREVER);

		frame_addr = msg.frame_addr;
		frame_length = msg.frame_length;

		/* COBS encode frm and then send out */
		enc_retv = cobs_encode(cobs_encoding_buffer, MAX_BM_FRAME_SIZE, frame_addr, frame_length);
		//LOG_INF( "Transmitting packet of length: %d", enc_retv.out_len);

		if (enc_retv.status == COBS_ENCODE_OK)
		{
			for ( n=0; n < MAX_SERIAL_DEV_COUNT; n++)
			{
				if (serial_dev[n] != NULL)
				{
					/* Send out UART one byte at a time */
					i = 0;
					while (i < enc_retv.out_len)
					{
						uart_poll_out(serial_dev[n], cobs_encoding_buffer[i++]);
					}
					uart_poll_out(serial_dev[n], 0x00);
				}
			}
		}
	}
}

static void bm_serial_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);

	if (uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			bm_serial_read_rx_byte(dev);
		}
	}
}

static void bm_serial_setup(void)
{
	uint8_t c;
	uint8_t i;

	for ( i = 0; i < MAX_SERIAL_DEV_COUNT; i++ )
	{
		if (serial_dev[i] != NULL)
		{
			uart_irq_rx_disable(serial_dev[i]);

			/* Drain the fifo */
			while (uart_fifo_read(serial_dev[i], &c, 1)) 
			{
				continue;
			}

			uart_irq_callback_set(serial_dev[i], bm_serial_isr);
			uart_irq_rx_enable(serial_dev[i]);
		}
	}
}

struct k_msgq* bm_serial_get_rx_msgq_handler(void)
{
	/* Simple getter for allowing ieee802154_bm_serial.c to listen to message queue */
	return &rx_queue;
}

/* Computes CRC16, stores in Tx Frame Buffer, and adds message to Queue for Tx Task  */
int bm_serial_frm_put(bm_frame_t* bm_frm)
{
	int retval = -1;
	uint16_t frame_length = bm_frm->frm_hdr.payload_length + sizeof(bm_frame_header_t);
	uint16_t computed_crc16 = crc16_ccitt(0, (uint8_t *) bm_frm, frame_length);
	//LOG_INF("CRC sent: %d", computed_crc16);

	if (k_msgq_num_free_get(&tx_queue))
	{
		/* Add frame to TX Contiguous Mem */
		memcpy(&tx_payload_buf[tx_payload_idx * MAX_BM_FRAME_SIZE], bm_frm, frame_length);
		/* Add CRC16 after Frame */
		memcpy(&tx_payload_buf[(tx_payload_idx * MAX_BM_FRAME_SIZE) + frame_length], &computed_crc16, sizeof(bm_crc_t));

		/* Update the frame length with CRC16 */
		frame_length += sizeof(bm_crc_t);

		/* Add msg to TX Message Queue (for TX Task to consume */ 
		bm_msg_t tx_msg = { .frame_addr = &tx_payload_buf[tx_payload_idx * MAX_BM_FRAME_SIZE], .frame_length = frame_length};
		retval = k_msgq_put(&tx_queue, &tx_msg, K_FOREVER);

		// Update index for storing next TX Payload
		tx_payload_idx++;
		if (tx_payload_idx >= NUM_BM_FRAMES)
		{
			tx_payload_idx = 0;
		}
	}
	else
	{
		LOG_ERR("TX MessageQueue full, dropping message!");
	}
	return retval;	
}

void bm_serial_init(void)
{
	static const struct device* _dev;
	uint8_t counter = 0;

	_dev = device_get_binding(CONFIG_BM_SERIAL_DEV_NAME_0);
	if (_dev != NULL) 
	{
		serial_dev[0] = _dev;
		counter++;
	}

	_dev = device_get_binding(CONFIG_BM_SERIAL_DEV_NAME_1);
	if (_dev != NULL) 
	{
		serial_dev[1] = _dev;
		counter++;
	}

	_dev = device_get_binding(CONFIG_BM_SERIAL_DEV_NAME_2);
	if (_dev != NULL) 
	{
		serial_dev[2] = _dev;
		counter++;
	}

	if (counter > 0)
	{
		bm_serial_setup();
	}
	else
	{
		LOG_ERR("At least one Serial Device should be defined in prj.conf");
	}

	k_thread_create(&rx_thread_data, bm_rx_stack,
			K_THREAD_STACK_SIZEOF(bm_rx_stack),
			(k_thread_entry_t)bm_serial_rx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);
	
	k_thread_create(&tx_thread_data, bm_tx_stack,
			K_THREAD_STACK_SIZEOF(bm_tx_stack),
			(k_thread_entry_t)bm_serial_tx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);
}
