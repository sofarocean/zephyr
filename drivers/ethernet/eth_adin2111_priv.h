/*
 * Copyright (c) 2020 DENX Software Engineering GmbH
 *               Lukasz Majewski <lukma@denx.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/ethernet.h>

#ifndef __ETH_ADIN2111_PRIV_H__
#define __ETH_ADIN2111_PRIV_H__

#define ADIN_MAC_ADDR_0 (0x00)
#define ADIN_MAC_ADDR_1 (0x0A)
#define ADIN_MAC_ADDR_2 (0x83)

/* Extra 4 bytes for FCS and 2 bytes for the frame header */
#define MAX_FRAME_BUF_SIZE  (MAX_FRAME_SIZE + 4 + 2)

/* Zephyr Specific structs */
struct adin2111_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec interrupt;
    struct gpio_dt_spec reset;
    void (*config_func)(void);
    uint8_t full_duplex;
    int32_t timeout;
};

struct adin2111_runtime {
    struct net_if *iface;
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ETH_ADIN2111_SERVICE_THREAD_STACK_SIZE);
    struct k_thread thread;
    uint8_t mac_addr[6];
    struct gpio_callback gpio_cb;
    struct k_sem tx_sem;
    struct k_sem int_sem;
    void (*generate_mac)(uint8_t *mac);
    uint8_t buf[NET_ETH_MAX_FRAME_SIZE];
};

#endif /* __ETH_ADIN2111_PRIV_H__ */