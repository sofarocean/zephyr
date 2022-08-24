/** @file
 *  @brief BM STM32 UART PHY driver private header.
 *
 *  L2 Driver for Bristlemouth network interface that targets muxed STM32 BM UART PHY interfaces
 */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_STM32_HAL_PRIV_H_
#define ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_STM32_HAL_PRIV_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/bristlemouth.h>

#include <zephyr/sys/spsc_pbuf.h>

#include "bm_rx_frame.h"
#include "bm_tx_frame.h"

#define SOFAR_OUI_B0                (0xF0)
#define SOFAR_OUI_B1                (0x5E)
#define SOFAR_OUI_B2                (0xA5)

#define BM_STM32_HAL_MTU            (NET_BM_MTU)
#define BM_STM32_HAL_FRAME_SIZE_MAX (BM_STM32_HAL_MTU + 18)

#define BM_CRC_SIZE                 (sizeof(uint32_t))

#ifdef BM_STM32_HAL_ENABLE_MANCHESTER_CODING
#define BM_BYTE_SIZE_SCALAR (2)
#else
#define BM_BYTE_SIZE_SCALAR (1)
#endif

#define BM_MIN_WIRE_FRAME_SIZE (BM_BYTE_SIZE_SCALAR + BM_BYTE_SIZE_SCALAR * BM_CRC_SIZE)
#define BM_MAX_WIRE_FRAME_SIZE ((BM_BYTE_SIZE_SCALAR * BM_STM32_HAL_FRAME_SIZE_MAX) + (BM_BYTE_SIZE_SCALAR * BM_CRC_SIZE))

/* Device run time data */

struct bm_tx_dma_buf {
    uint8_t data[ BM_MAX_WIRE_FRAME_SIZE + CONFIG_BM_STM32_HAL_PREAMBLE_LEN ];
    size_t len;
};
struct bm_stm32_hal_dev_data {
	struct net_if*                  iface;
	uint8_t                         mac_addr[6];
    bool                            link_up;

    // TX Data
    struct k_mutex                  tx_mutex;
    struct k_sem                    tx_int_sem;
    struct mpsc_pbuf_buffer         tx_mpsc;
    struct mpsc_pbuf_buffer_config  tx_mpsc_cfg;
    uint32_t                        tx_mpsc_buffer[ 8192 / sizeof(uint32_t) ]; // TODO: Make this come from config.
    struct bm_tx_dma_buf            tx_dma_buf[ CONFIG_BM_STM32_HAL_MAX_SERIAL_DEV_COUNT ];

    // RX Data
    struct k_sem                    rx_int_sem;
    struct mpsc_pbuf_buffer         enc_rx_mpsc;
    struct mpsc_pbuf_buffer_config  enc_rx_mpsc_cfg;
    uint32_t                        enc_rx_mpsc_buffer[ 8192 / sizeof(uint32_t) ]; // TODO: Make this come from config.
    uint32_t                        enc_rx_mpsc_drop_cnt;
    uint8_t                         dec_rx_buf[2*(CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE) + CONFIG_BM_STM32_HAL_PREAMBLE_LEN];

	K_KERNEL_STACK_MEMBER( rx_thread_stack, CONFIG_BM_STM32_HAL_RX_THREAD_STACK_SIZE );
	struct k_thread rx_thread;

    K_KERNEL_STACK_MEMBER( tx_thread_stack, CONFIG_BM_STM32_HAL_TX_THREAD_STACK_SIZE );
	struct k_thread tx_thread;
};

typedef struct bm_ctx_t
{
    const struct device* serial_dev;
    struct k_sem sem;
    struct k_timer timer;
    uint32_t interframe_delay_us;

    uint8_t buf[8*CONFIG_BM_STM32_HAL_MAX_FRAME_SIZE];  // DMA buf
} bm_ctx_t;

#endif /* ZEPHYR_INCLUDE_DRIVERS_CONSOLE_BM_STM32_HAL_PRIV_H_ */
