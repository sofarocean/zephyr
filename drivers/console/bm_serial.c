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
#include <thread.h>
#include <drivers/uart.h>

#include <drivers/console/bm_serial.h>
#include <sys/printk.h>
#include "bm_serial.h"

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

/* Buffer for received Frame Payloads */
uint8_t rx_payload_buf[NUM_BM_FRAMES * MAX_BM_FRAME_SIZE];
uint8_t rx_payload_idx = 0;

/* RX/TX Threads and associated message Queues */
K_THREAD_STACK_DEFINE(tx_stack, TASK_STACK_SIZE);
K_MSGQ_DEFINE(tx_queue, sizeof(bm_msg_t), NUM_BM_FRAMES, 4);
K_THREAD_STACK_DEFINE(rx_stack, TASK_STACK_SIZE);
K_MSGQ_DEFINE(rx_queue, sizeof(bm_msg_t), NUM_BM_FRAMES, 4);

/* Semaphore for ISR and RX_Task to produce and consume to double decode buffer */
K_SEM_DEFINE(decode_sem, 0, 1);

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
			// Write length of frame
			cobs_decoding_buf[write_buf_idx].length = decode_buf_off;

			/* Flip idx of read and write buffers */
			read_buf_idx = write_buf_idx;
			write_buf_idx = 1 - write_buf_idx;

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

	return (size_t)(encode - cobs_encoding_buffer);
}

/* Allows ieee802154_uart_pipe.c to notify the tx_task that a complete frame
   is ready to be transmitted over UART. */
int bm_serial_msg_put(bm_msg_t bm_msg)
{
	// Should this wait forever?
	return k_msgq_put(&tx_queue, &bm_msg, K_FOREVER);	
}

/**
 * BM Serial RX Thread
 */
static void bm_serial_rx_thread(void)
{
	unsigned int key;
	uint16_t computed_crc16;
	uint16_t received_crc16;
	uint16_t frame_length;
	uint16_t payload_length;
	LOG_DBG("BM Serial RX thread started");

	while (1)
	{
		// Wait on Producer to finish writing out decoded frame
		k_sem_take(&decode_sem, K_FOREVER);

		// Disable interrupts
		key = irq_lock();

		frame_length = cobs_decoding_buf[read_buf_idx].length;

		/* Verify CRC16 */
		computed_crc16 = crc16_ccitt(0, cobs_decoding_buf[read_buf_idx].buf, frame_length - sizeof(bm_crc_t));
		received_crc16 = cobs_decoding_buf[read_buf_idx].buf[frame_length - 2];
		received_crc16 |= (cobs_decoding_buf[read_buf_idx].buf[frame_length - 1] << 8);

		if (computed_crc16 != received_crc16)
		{
			LOG_ERR("CRC16 Mismatch in received frame, discarding\n");
			goto enable;
		}


		/* Add frame to RX Contiguous Mem */
		memcpy(&rx_payload_buf[rx_payload_idx * MAX_BM_FRAME_SIZE], cobs_decoding_buf[read_buf_idx].buf, frame_length);

		/* Add msg to RX Message Queue (for ieee802154_uart_pipe.c RX Task to consume */ 
		bm_msg_t rx_msg = { .frame_addr = &rx_payload_buf[rx_payload_idx * MAX_BM_FRAME_SIZE], .frame_length = frame_length};
		k_msgq_put(&tx_queue, &rx_msg, K_FOREVER);

		// Update index for storing next RX Payload
		rx_payload_idx++;
		if (rx_payload_idx <= NUM_BM_FRAMES)
		{
			rx_payload_idx = 0;
		}
enable:
		// Re-enable interrupts
		irq_unlock(key);
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

void bm_serial_init(void)
{
	uart_pipe_dev = device_get_binding(CONFIG_UART_PIPE_ON_DEV_NAME);

	if (uart_pipe_dev != NULL) 
	{
		bm_serial_setup(uart_pipe_dev);
	}

	k_thread_create(&rx_thread_data, rx_stack,
			K_THREAD_STACK_SIZEOF(rx_stack),
			(k_thread_entry_t)bm_serial_rx_thread,
			NULL, NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);
	
	k_thread_create(&tx_thread_data, tx_stack,
			K_THREAD_STACK_SIZEOF(tx_stack),
			(k_thread_entry_t)bm_serial_tx_thread,
			NULL, NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);
}
