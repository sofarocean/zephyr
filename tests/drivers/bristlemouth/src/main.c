/* main.c - OpenThread NCP */

/*
 * Copyright (c) 2020 Tridonic GmbH & Co KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <device.h>
#include <devicetree.h>

#include <logging/log.h>
#include <drivers/console/bm_serial.h>

#define LOG_MODULE_NAME bm_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

/* Basic test with 2 byte payload before delimiter */
static int basic_test_0 (void)
{
    int i;
    bm_test_ret_t ret;
    int success = 0;
    uint8_t test_buf[] = { 0x55, 0x55, 0X55, 0x55, 0x5A, 0xAA, 0xAA, 0xAA, 0xAA, 0xA9, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    uint8_t expected_out[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xA9, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

    for (i = 0; i < sizeof(test_buf); i++)
    {
        ret = bm_serial_process_byte(test_buf[i]);
    }

    if (ret.retval == 0 && ret.length == sizeof(expected_out))
    {
        for (i = 0; i < sizeof(expected_out); i++)
        {
            if (expected_out[i] != ret.buf_ptr[i])
            {
                goto out;
            }
        }
        success = 1;
    }

out:
    return success;
}

/* Missing preamble byte */
static int basic_test_1 (void)
{
    int i;
    bm_test_ret_t ret;
    int success = 0;
    uint8_t test_buf[] = { 0x55, 0x55, 0X55, 0x5A, 0xAA, 0xAA, 0xAA, 0xAA, 0xA9, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

    for (i = 0; i < sizeof(test_buf); i++)
    {
        ret = bm_serial_process_byte(test_buf[i]);
    }

    if (ret.retval != 0)
    {
        success = 1;
    }
    return success;
}

/* Test case with payload with 0xAA byte right before delimiter */
static int basic_test_2 (void)
{
    int i;
    bm_test_ret_t ret;
    int success = 0;
    uint8_t test_buf[] = { 0x55, 0x55, 0X55, 0x55, 0x5A, 0xAA, 0xAA, 0xAA, 0xAA, 0xA5, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x99, 0x99, 0x99, 0x99};
    uint8_t expected_out[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xA5, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x99, 0x99, 0x99, 0x99};

    for (i = 0; i < sizeof(test_buf); i++)
    {
        ret = bm_serial_process_byte(test_buf[i]);
    }

    if (ret.retval == 0 && ret.length == sizeof(expected_out))
    {
        for (i = 0; i < sizeof(expected_out); i++)
        {
            if (expected_out[i] != ret.buf_ptr[i])
            {
                goto out;
            }
        }
        success = 1;
    }

out:
    return success;
}

/* Missing delimiter byte */
static int basic_test_3 (void)
{
    int i;
    bm_test_ret_t ret;
    int success = 0;
    uint8_t test_buf[] = { 0x55, 0x55, 0X55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0xA9, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

    for (i = 0; i < sizeof(test_buf); i++)
    {
        ret = bm_serial_process_byte(test_buf[i]);
    }

    if (ret.retval != 0)
    {
        success = 1;
    }
    return success;
}

/* Missing delimiter byte and following byte is */
static int basic_test_4 (void)
{
    int i;
    bm_test_ret_t ret;
    int success = 0;
    uint8_t test_buf[] = { 0x55, 0x55, 0X55, 0x55, 0x5A, 0xAA, 0xAA, 0xAA, 0xA9, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

    for (i = 0; i < sizeof(test_buf); i++)
    {
        ret = bm_serial_process_byte(test_buf[i]);
    }

    if (ret.retval != 0)
    {
        success = 1;
    }
    return success;
}


void main(void)
{
    int ret;
    LOG_INF( "Testing the BM Serial driver" );

	bm_serial_init();

	while (1)
    {
        ret = basic_test_0();
        if(ret == 1)
        {
            printk("Basic Test 0 passed\n");
        }
        else
        {
            printk("Basic Test 0 failed\n");
        }

        ret = basic_test_1();
        if(ret == 1)
        {
            printk("Basic Test 1 passed\n");
        }
        else
        {
            printk("Basic Test 1 failed\n");
        }
        
        ret = basic_test_2();
        if(ret == 1)
        {
            printk("Basic Test 2 passed\n");
        }
        else
        {
            printk("Basic Test 2 failed\n");
        }

        ret = basic_test_3();
        if(ret == 1)
        {
            printk("Basic Test 3 passed\n");
        }
        else
        {
            printk("Basic Test 3 failed\n");
        }

        ret = basic_test_0();
        if(ret == 1)
        {
            printk("Basic Test 0 passed\n");
        }
        else
        {
            printk("Basic Test 0 failed\n");
        }

        ret = basic_test_4();
        if(ret == 1)
        {
            printk("Basic Test 4 passed\n");
        }
        else
        {
            printk("Basic Test 4 failed\n");
        }

        while(1);
    }
}