/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_ethernet_adin2111, LOG_LEVEL_DBG);

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_core.h>
#include <zephyr/kernel.h>

#include <zephyr/net/net_core.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/crc.h>
#include <fcntl.h>

static struct net_if *iface;

static const struct gpio_dt_spec user2 = GPIO_DT_SPEC_GET(DT_ALIAS(user2), gpios);
static const struct gpio_dt_spec user3 = GPIO_DT_SPEC_GET(DT_ALIAS(user3), gpios);

K_THREAD_STACK_DEFINE(dummy_stack_area, 512);
struct k_thread dummy_thread_data;

K_THREAD_STACK_DEFINE(main_tx_stack_area, 2048);
struct k_thread main_tx_thread_data;

int sock;
struct sockaddr* addr;
struct sockaddr_in6 addr6;

void join_well_known_multicast_groups(struct net_if *iface)
{
    struct in6_addr ll_all_nodes_addr;
    struct in6_addr rl_all_nodes_addr;
    struct net_if_mcast_addr *maddr;

    net_ipv6_addr_create(&ll_all_nodes_addr, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001);
    net_ipv6_addr_create(&rl_all_nodes_addr, 0xff03, 0, 0, 0, 0, 0, 0, 0x0001);
    
    // Add and join link-local
    maddr = net_if_ipv6_maddr_lookup(&ll_all_nodes_addr, &iface);
    if (maddr && net_if_ipv6_maddr_is_joined(maddr)) {
        // Already joined
    }

    if (!maddr) {
        maddr = net_if_ipv6_maddr_add(iface, &ll_all_nodes_addr);
        if (!maddr) {
            NET_ERR( "Failed to add LL multicast address" );
        }
    }

    net_if_ipv6_maddr_join(maddr);

    // Add and join realm
    maddr = net_if_ipv6_maddr_lookup(&rl_all_nodes_addr, &iface);
    if (maddr && net_if_ipv6_maddr_is_joined(maddr)) {
        // Already joined
    }

    if (!maddr) {
        maddr = net_if_ipv6_maddr_add(iface, &rl_all_nodes_addr);
        if (!maddr) {
            NET_ERR( "Failed to add RL multicast address" );
        }
    }

    net_if_ipv6_maddr_join(maddr);
}

/* Dummy Thread */
// static void dummy_thread(void) {
//     k_thread_custom_data_set((void *) &user3);
//     while (1) {
//         k_sleep(K_USEC(100));
//     }
// }

/* Main TX Thread */
static void main_tx_thread(void) {
    int retval;
    static char send_buf[] = "Lorem ipsum dolor sit amet consectetur adipiscing elit mi primis, \
                            convallis augue fringilla rhoncus quis mauris nam pharetra egestas, \
                            iaculis ligula ultrices himenaeos mus non enim taciti. Sed penatibus \
                            nisi duis odio sociosqu nascetur ultricies, fames dui tellus orci \
                            sollicitudin congue neque porta, praesent pellentesque quis potenti \
                            euismod augue ultrices himenaeos mus non enim taciti. Sed penatibus \
                            nisi duis odio sociosqu nascetur ultricies, fames dui tellus orci \
                            sollicitudin congue neque porta, praesent pellentesque quis potenti \
                            euismod augue ultrices himenaeos mus non enim taciti. Sed penatibus \
                            nisi duis odio sociosqu nascetur ultricies, fames dui tellus orci \
                            sollicitudin congue fames duis odio sociosqu nascetur ultricies, fames dui tellus orci \
                            sollicitudin congue fames duis odio sociosqu nascetur ultricies, fames dui tellus orci \
                            sollicitudin congue fames duis odio sociosqu nascetur ultricies, fames dui tellus orci \
                            sollicitudin congue fames duis odio sociosqu nascetur ultricies, fames dui tellus orci fames dui tell";

        uint8_t counter = 0;
        while (1) {
            send_buf[0] = counter;
            counter++;
            retval = zsock_sendto(sock, send_buf, sizeof(send_buf), 0, addr, sizeof(*addr));
            if (retval < 0) {
                LOG_ERR("Failed to send");
            } else {
                //printk(".");
            }
            k_sleep(K_USEC(350));
        }
}



/* This application itself does nothing as there is net-shell that can be used
 * to monitor things.
 */
void main(void)
{
    int retval;
    LOG_INF("Testing Ethernet ADIN2111 driver");

    iface = net_if_get_by_index(1);
    join_well_known_multicast_groups(iface);

    addr6.sin6_family = AF_INET6;
    addr6.sin6_port = htons(2222);
    zsock_inet_pton(AF_INET6, "ff03::1", &addr6.sin6_addr);

    addr = (struct sockaddr *)&addr6;

    sock = zsock_socket(addr->sa_family, SOCK_DGRAM, IPPROTO_UDP);
    if( sock < 0 ) {
        LOG_ERR("Failed to create UDP socket: %d", errno);
    }

    retval = zsock_bind( sock, addr, sizeof(addr6) );
    if (retval < 0) {
        LOG_ERR("Cannot bind to UDP addr/port: %d", errno);
        while(1);
    } else {
        LOG_INF( "Bound" );
    }

    static char recv_buf[1411];

    uint8_t recv_counter;
    uint8_t last_counter = 0;
    bool first = true;

    // k_thread_create(&dummy_thread_data, dummy_stack_area,
    //                 K_THREAD_STACK_SIZEOF(dummy_stack_area),
    //                 (k_thread_entry_t)dummy_thread,
    //                 NULL, NULL, NULL,
    //                 K_PRIO_COOP(CONFIG_ETH_ADIN2111_BSP_THREAD_PRIO),
    //                 0, K_NO_WAIT);
    // k_thread_name_set(&dummy_thread_data, "adin2111_dummy_thread");

    k_thread_create(&main_tx_thread_data, main_tx_stack_area,
                    K_THREAD_STACK_SIZEOF(main_tx_stack_area),
                    (k_thread_entry_t)main_tx_thread,
                    NULL, NULL, NULL,
                    K_PRIO_COOP(16),
                    0, K_NO_WAIT);
    k_thread_name_set(&main_tx_thread_data, "main_tx_thread");

    //k_thread_custom_data_set((void *) &user3);

    while (1)
    {
        retval = zsock_recv(sock, recv_buf, sizeof(recv_buf), 0);
        if(retval == -EAGAIN) {
            LOG_INF( "Timeout" );
        } else if (retval < 0) {
            LOG_ERR( "Failed to recv" );
        } else {
            if (first) {
                gpio_pin_configure_dt(&user2, GPIO_OUTPUT_ACTIVE);
		        gpio_pin_configure_dt(&user2, GPIO_OUTPUT_INACTIVE);
                first = false;
                last_counter = recv_buf[0];
                gpio_pin_configure_dt(&user2, GPIO_OUTPUT_ACTIVE);
            } else {
                recv_counter = recv_buf[0];
                //printk("%d ", recv_counter);
                if (recv_counter != (uint8_t) (last_counter + 1)) {
                    LOG_ERR("Missed a frame. Counter: %d Last Counter: %d", recv_counter, last_counter);
                } else {
                    gpio_pin_configure_dt(&user2, GPIO_OUTPUT_ACTIVE);
		            gpio_pin_configure_dt(&user2, GPIO_OUTPUT_INACTIVE);
                    gpio_pin_configure_dt(&user2, GPIO_OUTPUT_ACTIVE);
                }
                last_counter = recv_counter;
            }
        }
    }
}