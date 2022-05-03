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
#include <drivers/gpio.h>

static const struct device* serial_dev[3] = {0};

static struct k_thread tx_thread_data;
static struct k_thread rx_thread_data;

/* Buffer to store incoming Rx Manchester-encoded data.*/
volatile bm_rx_t encoded_rx_buf[2];

/* Read and Write Pointers to the double buffer */
volatile uint8_t write_buf_idx = 0;
volatile uint8_t read_buf_idx = 1;

/* Individual buffer counter */
uint8_t encoded_rx_ctr = 0;

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

const uint16_t MAN_ENCODE_TABLE[256] = 
{
    0xAAAA, 0xAAA9, 0xAAA6, 0xAAA5, 0xAA9A, 0xAA99, 0xAA96, 0xAA95, 0xAA6A, 0xAA69, 0xAA66, 0xAA65, 0xAA5A, 0xAA59, 0xAA56, 0xAA55,
    0xA9AA, 0xA9A9, 0xA9A6, 0xA9A5, 0xA99A, 0xA999, 0xA996, 0xA995, 0xA96A, 0xA969, 0xA966, 0xA965, 0xA95A, 0xA959, 0xA956, 0xA955,
    0xA6AA, 0xA6A9, 0xA6A6, 0xA6A5, 0xA69A, 0xA699, 0xA696, 0xA695, 0xA66A, 0xA669, 0xA666, 0xA665, 0xA65A, 0xA659, 0xA656, 0xA655,
    0xA5AA, 0xA5A9, 0xA5A6, 0xA5A5, 0xA59A, 0xA599, 0xA596, 0xA595, 0xA56A, 0xA569, 0xA566, 0xA565, 0xA55A, 0xA559, 0xA556, 0xA555,
    0x9AAA, 0x9AA9, 0x9AA6, 0x9AA5, 0x9A9A, 0x9A99, 0x9A96, 0x9A95, 0x9A6A, 0x9A69, 0x9A66, 0x9A65, 0x9A5A, 0x9A59, 0x9A56, 0x9A55,
    0x99AA, 0x99A9, 0x99A6, 0x99A5, 0x999A, 0x9999, 0x9996, 0x9995, 0x996A, 0x9969, 0x9966, 0x9965, 0x995A, 0x9959, 0x9956, 0x9955,
    0x96AA, 0x96A9, 0x96A6, 0x96A5, 0x969A, 0x9699, 0x9696, 0x9695, 0x966A, 0x9669, 0x9666, 0x9665, 0x965A, 0x9659, 0x9656, 0x9655,
    0x95AA, 0x95A9, 0x95A6, 0x95A5, 0x959A, 0x9599, 0x9596, 0x9595, 0x956A, 0x9569, 0x9566, 0x9565, 0x955A, 0x9559, 0x9556, 0x9555,
    0x6AAA, 0x6AA9, 0x6AA6, 0x6AA5, 0x6A9A, 0x6A99, 0x6A96, 0x6A95, 0x6A6A, 0x6A69, 0x6A66, 0x6A65, 0x6A5A, 0x6A59, 0x6A56, 0x6A55,
    0x69AA, 0x69A9, 0x69A6, 0x69A5, 0x699A, 0x6999, 0x6996, 0x6995, 0x696A, 0x6969, 0x6966, 0x6965, 0x695A, 0x6959, 0x6956, 0x6955,
    0x66AA, 0x66A9, 0x66A6, 0x66A5, 0x669A, 0x6699, 0x6696, 0x6695, 0x666A, 0x6669, 0x6666, 0x6665, 0x665A, 0x6659, 0x6656, 0x6655,
    0x65AA, 0x65A9, 0x65A6, 0x65A5, 0x659A, 0x6599, 0x6596, 0x6595, 0x656A, 0x6569, 0x6566, 0x6565, 0x655A, 0x6559, 0x6556, 0x6555,
    0x5AAA, 0x5AA9, 0x5AA6, 0x5AA5, 0x5A9A, 0x5A99, 0x5A96, 0x5A95, 0x5A6A, 0x5A69, 0x5A66, 0x5A65, 0x5A5A, 0x5A59, 0x5A56, 0x5A55,
    0x59AA, 0x59A9, 0x59A6, 0x59A5, 0x599A, 0x5999, 0x5996, 0x5995, 0x596A, 0x5969, 0x5966, 0x5965, 0x595A, 0x5959, 0x5956, 0x5955,
    0x56AA, 0x56A9, 0x56A6, 0x56A5, 0x569A, 0x5699, 0x5696, 0x5695, 0x566A, 0x5669, 0x5666, 0x5665, 0x565A, 0x5659, 0x5656, 0x5655,
    0x55AA, 0x55A9, 0x55A6, 0x55A5, 0x559A, 0x5599, 0x5596, 0x5595, 0x556A, 0x5569, 0x5566, 0x5565, 0x555A, 0x5559, 0x5556, 0x5555,
};

const int8_t MAN_DECODE_TABLE[256] = 
{
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, 0xF, 0xE, -1, -1, 0xD, 0xC, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, 0xB, 0xA, -1, -1, 0x9, 0x8, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, 0x7, 0x6, -1, -1, 0x5, 0x4, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, 0x3, 0x2, -1, -1, 0x1, 0x0, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

#define USER0_NODE DT_ALIAS(user0)
#define USER1_NODE DT_ALIAS(user1)

/* These map to D3/D4 on the L496ZG Nucleo*/
static const struct gpio_dt_spec user0 = GPIO_DT_SPEC_GET(USER0_NODE, gpios);
static const struct gpio_dt_spec user1 = GPIO_DT_SPEC_GET(USER1_NODE, gpios);

static bm_parse_ret_t bm_serial_align(uint8_t rx_byte)
{
    bm_parse_ret_t ret = {.new_state=BM_ALIGN, .success=0};
	static uint8_t preamble_ctr = 0;

	if (rx_byte == BM_PREAMBLE_VAL) 
	{
		preamble_ctr++;
	}
	else if (preamble_ctr >= BM_PREAMBLE_LEN && rx_byte == BM_DELIMITER_VAL) 
	{
		/* We have aligned with the Preamble and Delimiter*/
		preamble_ctr = 0;
		ret.new_state = BM_COLLECT_HEADER;
		ret.success = 1;
	}
	else
	{
		/* We reach this case if we receive non-preamble bytes when we expect them */
		LOG_ERR("Expected Preamble, but got errant bytes");

		/* Reset counter */
		preamble_ctr = 0;
	}
    return ret;
}

bm_ret_t bm_serial_process_byte(uint8_t byte)
{
	bm_ret_t test_ret = {.retval=-EINPROGRESS, .length=0, .buf_ptr=NULL};
	bm_parse_ret_t ret;
	static uint16_t payload_ctr = 0;
	static bm_parse_state_t _state = BM_ALIGN;
	static uint8_t decoded_version = 0;
	static uint8_t decoded_type = 0;
	static uint16_t decoded_length = 0;
	static uint16_t decoded_crc = 0;
	int8_t decoded_nibble_lsb = 0;
	int8_t decoded_nibble_msb = 0;
	uint16_t computed_crc16 = 0;
	bm_frame_header_t rx_header;

	switch (_state)
	{
		case BM_ALIGN:

			/* Reset static variables */
			decoded_length = 0;
			decoded_version = 0;
			decoded_type = 0;
			decoded_crc = 0;
			payload_ctr = 0;

			ret = bm_serial_align(byte);
			_state = ret.new_state;
			break;
		case BM_COLLECT_HEADER:
			encoded_rx_buf[write_buf_idx].buf[encoded_rx_ctr++] = byte;

			/* Collect until we have a full frame header */
			if (encoded_rx_ctr < (2*sizeof(bm_frame_header_t)))
			{
				break;
			}
			/* Start with version */
			decoded_nibble_lsb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[offsetof(bm_frame_header_t, version) * 2]];
			decoded_nibble_msb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[(offsetof(bm_frame_header_t, version) * 2) + 1]];
			if (decoded_nibble_lsb >= 0 || decoded_nibble_msb >= 0 )
			{
				decoded_version |= (((uint8_t) decoded_nibble_lsb) & 0xF);
				decoded_version |= ((((uint8_t) decoded_nibble_msb) & 0xF) << 4);
				rx_header.version = decoded_version;
			}
			else
			{
				LOG_ERR("Invalid Manchester value for BM version");
				/* Reset RX buffer index */
				encoded_rx_ctr = 0;
				/* Reset bm parse state */
				_state = BM_ALIGN;
				break;
			}

			/* Next decoded BM Payload Type */
			decoded_nibble_lsb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[offsetof(bm_frame_header_t, payload_type) * 2]];
			decoded_nibble_msb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[(offsetof(bm_frame_header_t, payload_type) * 2) + 1]];
			if (decoded_nibble_lsb >= 0 || decoded_nibble_msb >= 0 )
			{
				decoded_type |= (((uint8_t) decoded_nibble_lsb) & 0xF);
				decoded_type |= ((((uint8_t) decoded_nibble_msb) & 0xF) << 4);
				rx_header.payload_type = decoded_type;
			}
			else
			{
				LOG_ERR("Invalid Manchester value for BM payload type");
				/* Reset RX buffer index */
				encoded_rx_ctr = 0;
				/* Reset bm parse state */
				_state = BM_ALIGN;
				break;
			}

			/* Now decode length lower bits */
			decoded_nibble_lsb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[offsetof(bm_frame_header_t, payload_length) * 2]];
			decoded_nibble_msb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[(offsetof(bm_frame_header_t, payload_length) * 2) + 1]];
			if (decoded_nibble_lsb >= 0 || decoded_nibble_msb >= 0 )
			{
				decoded_length |= (((uint8_t) decoded_nibble_lsb) & 0xF);
				decoded_length |= ((((uint8_t) decoded_nibble_msb) & 0xF) << 4);
			}
			else
			{
				LOG_ERR("Invalid Manchester value for MSB of frame length");
				/* Reset RX buffer index */
				encoded_rx_ctr = 0;
				/* Reset bm parse state */
				_state = BM_ALIGN;
				break;
			}

			/* Now decode length upper bits */
			decoded_nibble_lsb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[(offsetof(bm_frame_header_t, payload_length) * 2) + 2]];
			decoded_nibble_msb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[(offsetof(bm_frame_header_t, payload_length) * 2) + 3]];
			if (decoded_nibble_lsb >= 0 || decoded_nibble_msb >= 0 )
			{
				decoded_length |= ((((uint8_t) decoded_nibble_lsb) & 0xF) << 8);
				decoded_length |= ((((uint8_t) decoded_nibble_msb) & 0xF) << 12);
				rx_header.payload_length = decoded_length;
			}
			else
			{
				LOG_ERR("Invalid Manchester value for MSB of frame length");
				/* Reset RX buffer index */
				encoded_rx_ctr = 0;
				/* Reset bm parse state */
				_state = BM_ALIGN;
				break;
			}

			/* Finally decode header CRC lower bits */
			decoded_nibble_lsb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[offsetof(bm_frame_header_t, header_crc) * 2]];
			decoded_nibble_msb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[(offsetof(bm_frame_header_t, header_crc) * 2) + 1]];
			if (decoded_nibble_lsb >= 0 || decoded_nibble_msb >= 0 )
			{
				decoded_crc |= (((uint8_t) decoded_nibble_lsb) & 0xF);
				decoded_crc |= ((((uint8_t) decoded_nibble_msb) & 0xF) << 4);
			}
			else
			{
				LOG_ERR("Invalid Manchester value for MSB of frame length");
				/* Reset RX buffer index */
				encoded_rx_ctr = 0;
				/* Reset bm parse state */
				_state = BM_ALIGN;
				break;
			}

			/* Now decode header CRC upper bits */
			decoded_nibble_lsb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[(offsetof(bm_frame_header_t, header_crc) * 2) + 2]];
			decoded_nibble_msb = MAN_DECODE_TABLE[encoded_rx_buf[write_buf_idx].buf[(offsetof(bm_frame_header_t, header_crc) * 2) + 3]];
			if (decoded_nibble_lsb >= 0 || decoded_nibble_msb >= 0 )
			{
				decoded_crc |= ((((uint8_t) decoded_nibble_lsb) & 0xF) << 8);
				decoded_crc |= ((((uint8_t) decoded_nibble_msb) & 0xF) << 12);
				rx_header.header_crc = decoded_crc;
			}
			else
			{
				LOG_ERR("Invalid Manchester value for MSB of frame length");
				/* Reset RX buffer index */
				encoded_rx_ctr = 0;
				/* Reset bm parse state */
				_state = BM_ALIGN;
				break;
			}

			/* Verify CRC16 */
			computed_crc16 = crc16_ccitt(0, (uint8_t *) &rx_header, sizeof(bm_frame_header_t) - sizeof(bm_crc_t));

			if (computed_crc16 != rx_header.header_crc)
			{
				LOG_ERR("Header CRC16 received: %d vs. computed: %d, discarding\n", rx_header.header_crc, computed_crc16);
				/* Reset RX buffer index */
				encoded_rx_ctr = 0;
				/* Reset bm parse state */
				_state = BM_ALIGN;
				break;
			}

			/* Make sure versions match */
			if (decoded_version == BM_V0)
			{
				_state = BM_COLLECT_PAYLOAD;
			}
			else
			{
				/* Reset RX buffer index */
				encoded_rx_ctr = 0;
				/* Reset bm parse state */
				_state = BM_ALIGN;
				break;
			}
			break;
		case BM_COLLECT_PAYLOAD:
			encoded_rx_buf[write_buf_idx].buf[encoded_rx_ctr++] = byte;
			payload_ctr++;

			if (payload_ctr == ((2* decoded_length) + (2*sizeof(bm_crc_t))))
			{
				/* First check if the RX_Task has finished processing read_buf */
				if ( k_sem_take(&processing_sem, K_NO_WAIT) != 0 )
				{
					LOG_ERR("RX Task has not finished processing data. Overwriting current index of A/B Buffer.");
				}
				else
				{
					/* Set length of received frame */
					encoded_rx_buf[write_buf_idx].length = encoded_rx_ctr;
					test_ret.length = encoded_rx_ctr;
					test_ret.buf_ptr = encoded_rx_buf[write_buf_idx].buf;

					/* Reset RX buffer index */
					encoded_rx_ctr = 0;

					/* Reset bm parse state */
					_state = BM_ALIGN;

					/* Flip idx of read and write buffers */
					read_buf_idx = write_buf_idx;
					write_buf_idx = 1 - write_buf_idx;

					/* Notify RX Task that frame is ready to be processed */
					k_sem_give(&decode_sem);
					test_ret.retval = 0;
				}
			}
			break;
		default:
			break;
	}
	return test_ret;
}

/**
 * BM Serial RX Thread
 */
static void bm_serial_rx_thread(void)
{
	uint16_t computed_crc16;
	uint16_t received_crc16;
	int retval;
	int i;
	/* Buffer to store Manchester Decoded RX data */
	uint8_t man_decode_buf[ MAX_BM_FRAME_SIZE ] = {0};
	uint8_t man_decode_len = 0;
	int8_t decoded_nibble = 0;

	LOG_DBG("BM Serial RX thread started");

	while (1)
	{
		/* Wait on Producer to finish writing out decoded frame */
		k_sem_take(&decode_sem, K_FOREVER);

		memset(man_decode_buf, 0, sizeof(man_decode_buf));

		/* Decode manchester - Assumption that index is even */
		man_decode_len = encoded_rx_buf[read_buf_idx].length/2;
		for ( i=0; i < man_decode_len; i++ )
		{
			decoded_nibble = MAN_DECODE_TABLE[encoded_rx_buf[read_buf_idx].buf[(2*i)]];
			if (decoded_nibble >= 0)
			{
				man_decode_buf[i] |= (((uint8_t) decoded_nibble) & 0xF);
			}
			else
			{
				LOG_ERR("Invalid Manchester value");
			}

			decoded_nibble = MAN_DECODE_TABLE[encoded_rx_buf[read_buf_idx].buf[(2*i)+1]];
			if (decoded_nibble >= 0)
			{
				man_decode_buf[i] |= ((((uint8_t) decoded_nibble) & 0xF) << 4);
			}
			else
			{
				LOG_ERR("Invalid Manchester value");
			}
		}

		if (man_decode_len == 0)
		{
			LOG_ERR("Received Frame Length of 0. Skipping");
			k_sem_give(&processing_sem);
			continue;
		}

		/* Verify CRC16 (Bristlemouth Packet = header + payload)*/
		computed_crc16 = crc16_ccitt(0, man_decode_buf, man_decode_len - sizeof(bm_crc_t));
		received_crc16 = man_decode_buf[man_decode_len - 2];
		received_crc16 |= (man_decode_buf[man_decode_len - 1] << 8);

		if (computed_crc16 != received_crc16)
		{
			LOG_ERR("CRC16 received: %d vs. computed: %d, discarding\n", received_crc16, computed_crc16);
			k_sem_give(&processing_sem);
			continue;
		}

		/* Update frame length with CRC16 removal */
		man_decode_len -= sizeof(bm_crc_t);

		if(k_msgq_num_free_get(&rx_queue))
		{
			/* Add frame to RX Contiguous Mem */
			memcpy(&rx_payload_buf[rx_payload_idx * MAX_BM_FRAME_SIZE], man_decode_buf, man_decode_len);

			/* Add msg to RX Message Queue (for ieee802154_bm_serial.c RX Task to consume */ 
			bm_msg_t rx_msg = { .frame_addr = &rx_payload_buf[rx_payload_idx * MAX_BM_FRAME_SIZE], .frame_length = man_decode_len};
			retval = k_msgq_put(&rx_queue, &rx_msg, K_FOREVER);
			if (retval)
			{
				LOG_ERR("Message could not be added to Queue");
			}

			/* Update index for storing next RX Payload */
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
	uint16_t i;
	uint8_t n;
	uint16_t encoded_val;

	while (1) 
	{
		k_msgq_get(&tx_queue, &msg, K_FOREVER);

		frame_addr = msg.frame_addr;
		frame_length = msg.frame_length;

		for ( n=0; n < MAX_SERIAL_DEV_COUNT; n++)
		{
			if (serial_dev[n] != NULL)
			{
				/* Send out preamble */
				for ( i=0; i < BM_PREAMBLE_LEN; i++)
				{
					uart_poll_out(serial_dev[n], BM_PREAMBLE_VAL);
				}
				
				/* Send out delimiter */
				uart_poll_out(serial_dev[n], BM_DELIMITER_VAL);

				/* Send out UART one byte at a time */
				for ( i=0; i < frame_length; i++)
				{
					encoded_val = MAN_ENCODE_TABLE[frame_addr[i]];
					uart_poll_out(serial_dev[n], (uint8_t) (encoded_val & 0xFF));
					uart_poll_out(serial_dev[n], (uint8_t) ((encoded_val >> 8) & 0xFF));
				}
			}
		}
	}
}

static void bm_serial_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t byte;

	uart_irq_update(dev);

	if (uart_irq_is_pending(dev) && uart_irq_rx_ready(dev)) 
	{
		if ( uart_fifo_read(dev, &byte, 1) )
		{
			bm_serial_process_byte(byte);
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

/* Computes Bristlemouth Packet CRC16, stores in Tx Frame Buffer, and adds message to Queue for Tx Task  */
int bm_serial_frm_put(bm_frame_t* bm_frm)
{
	int retval = -1;
	uint16_t frame_length = bm_frm->frm_hdr.payload_length + sizeof(bm_frame_header_t);
	/* Computed on the entire packet, using CCITT */
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

		/* Update index for storing next TX Payload */
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

	/* Checking/Setting up GPIO Debug Pins */
	if (!device_is_ready(user0.port))
    {
		LOG_ERR("GPIO 0 Port not ready");
	}

    if (!device_is_ready(user1.port))
    {
		LOG_ERR("GPIO 1 Port not ready");
	}

	int ret = gpio_pin_configure_dt(&user0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
    {
		LOG_ERR("GPIO 0 unable to be configured\n");
	}

    ret = gpio_pin_configure_dt(&user1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
    {
		LOG_ERR("GPIO 1 unable to be configured\n");
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
