/* main.c - Bristlemouth L2 Network Test */

/*
 * Copyright (c) 2022 Sofar Ocean Technologies
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


#include <zephyr/zephyr.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/socket.h>

#define LOG_MODULE_NAME bm_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

static char recv_buf[1500];
static char str_data[256];

void main(void)
{
    LOG_INF( "Testing the BM Net Driver" );

    int ret;
    int sock;
    struct sockaddr_in6 addr6;

    addr6.sin6_family = AF_INET6;
    addr6.sin6_port = htons(9999);
    zsock_inet_pton(AF_INET6, "ff03::1", &addr6.sin6_addr);

    struct sockaddr* addr = (struct sockaddr *)&addr6;

    sock = zsock_socket(addr->sa_family, SOCK_DGRAM, IPPROTO_UDP);
    if( sock < 0 ) 
    {
		LOG_ERR("Failed to create UDP socket: %d", errno);
	}

    ret = zsock_bind( sock, addr, sizeof(addr6) );
    if (ret < 0) {
		LOG_ERR("Cannot bind to UDP addr/port: %d", errno);
	}
    else
    {
        LOG_INF( "Bound" );
    }

    while (1)
    {
        int received;
        LOG_INF( "Waiting for data" );
	    received = zsock_recv(sock, recv_buf, sizeof(recv_buf), 0);

        if(received == -EAGAIN) {
            LOG_INF( "Timeout" );
		} else if (ret < 0) {
			LOG_ERR( "Failed to recv" );
		}
        else
        {
            memcpy( str_data, recv_buf, received );
            str_data[received] = 0;
            
            LOG_INF( "Got %d bytes: '%s'", received, str_data );

        }
    }
}