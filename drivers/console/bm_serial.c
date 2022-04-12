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

static const struct device *uart_pipe_dev;

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
	static volatile uint8_t code = 0xff;
	static volatile uint8_t block = 0;
	static volatile uint8_t wait_for_delimiter = 0;

	/* Frame is too long, must have missed something? */
	if (decode_buf_off >= MAX_BM_FRAME_SIZE)
	{
		/* Exceeded Max Allowable Packet Size 
		   Clear the Decode Buffer and wait for next 
		   Delimiter */
		decode_buf_off = 0;
		wait_for_delimiter = 1;
		goto out; 
	}

	if (wait_for_delimiter)
	{
		/* Start parsing again once delimiter has been found */
		if (!*rx_byte)
		{
			wait_for_delimiter = 0;
			goto reset;
		}
	}

	if (block)
	{
		cobs_decoding_buf[write_buf_idx].buf[decode_buf_off++] = *rx_byte; // Store received byte in decode buffer
	}
	else
	{
		if (code != 0xff)
		{
			cobs_decoding_buf[write_buf_idx].buf[decode_buf_off++] = 0;
		}
		block = code = *rx_byte; // Next block length
		if (!code)
		{
			// Write length of frame (without 0x00 delimiter)
			cobs_decoding_buf[write_buf_idx].length = decode_buf_off-1;

			/* First check if the RX_Task has finished processing read_buf */
			if ( k_sem_take(&processing_sem, K_NO_WAIT))
			{
				LOG_DBG("RX Task has not finished processing data. Overwriting current index of A/B Buffer.");
			}
			else
			{
				/* Flip idx of read and write buffers */
				read_buf_idx = write_buf_idx;
				write_buf_idx = 1 - write_buf_idx;
			}

			/* Notify RX Task that frame is ready to be processed */
			k_sem_give(&decode_sem);
			
			goto reset;
		}
	}
	block--;
	goto out;

reset:
	code = 0xff;
	block = 0;
	decode_buf_off = 0; // reset offset in decode buffer
out:
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
		got = uart_fifo_read(uart_pipe_dev, &byte, 1);
		if (got <= 0) 
		{
			break;
		}
		bm_parse_and_store(&byte);
	}
}

static size_t bm_serial_cobs_encode(const uint8_t *data, uint16_t length)
{
	uint8_t *encode = cobs_encoding_buffer; // Encoded byte pointer
	uint8_t *codep = encode++; // Output code pointer
	uint8_t code = 1; // Code value
	const uint8_t *byte;

	for (byte = (const uint8_t *)data; length--; ++byte)
	{
		if (*byte)
		{
			*encode++ = *byte, ++code;
		}

		if (!*byte || code == 0xff)
		{
			*codep = code, code = 1, codep = encode;
			if (!*byte || length)
			{
				++encode;
			}
		}
	}
	*codep = code; // Write final code value
	++encode;
	*encode = 0x00;

	return (size_t)(encode - cobs_encoding_buffer);
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

		/* Verify CRC16 */
		computed_crc16 = crc16_ccitt(0, cobs_decoding_buf[read_buf_idx].buf, frame_length - sizeof(bm_crc_t));
		received_crc16 = cobs_decoding_buf[read_buf_idx].buf[frame_length - 2];
		received_crc16 |= (cobs_decoding_buf[read_buf_idx].buf[frame_length - 1] << 8);

		if (computed_crc16 != received_crc16)
		{
			LOG_ERR("CRC16 Mismatch in received frame, discarding\n");
			goto signal;
		}

		/* Update frame length with CRC16 removal */
		frame_length -= sizeof(bm_crc_t);

		if(k_msgq_num_free_get(&rx_queue))
		{
			/* Add frame to RX Contiguous Mem */
			memcpy(&rx_payload_buf[rx_payload_idx * MAX_BM_FRAME_SIZE], cobs_decoding_buf[read_buf_idx].buf, frame_length);

			/* Add msg to RX Message Queue (for ieee802154_uart_pipe.c RX Task to consume */ 
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
signal:
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
	uint16_t cobs_length;
	uint16_t i;

	while (1) 
	{
		k_msgq_get(&tx_queue, &msg, K_FOREVER);

		frame_addr = msg.frame_addr;
		frame_length = msg.frame_length;

		/* COBS encode frm and then send out */
		cobs_length = bm_serial_cobs_encode((const uint8_t*) frame_addr, frame_length);

		/* Send out UART one byte at a time */
		i = 0;
		while (i < cobs_length)
		{
			uart_poll_out(uart_pipe_dev, cobs_encoding_buffer[i++]);
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

static void bm_serial_setup(const struct device *uart)
{
	uint8_t c;

	uart_irq_rx_disable(uart);
	uart_irq_tx_disable(uart);

	/* Drain the fifo */
	while (uart_fifo_read(uart, &c, 1)) 
	{
		continue;
	}

	uart_irq_callback_set(uart, bm_serial_isr);

	uart_irq_rx_enable(uart);
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
	uart_pipe_dev = device_get_binding(CONFIG_UART_PIPE_ON_DEV_NAME);

	if (uart_pipe_dev != NULL) 
	{
		bm_serial_setup(uart_pipe_dev);
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
