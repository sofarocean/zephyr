/*
 *---------------------------------------------------------------------------
 *
 * Copyright (c) 2020, 2021 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc.
 * and its licensors.By using this software you agree to the terms of the
 * associated Analog Devices Software License Agreement.
 *
 *---------------------------------------------------------------------------
 */

#include "adi_bsp.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <string.h>

#define RESET_DELAY       (1)
#define AFTER_RESET_DELAY (100)

static struct   gpio_callback gpio_cb;

static ADI_CB gpfIntCallback = NULL;
static void *gpIntCBParam = NULL;

static ADI_CB gpfSpiCallback = NULL;
static void *gpSpiCBParam = NULL;

K_THREAD_STACK_DEFINE(bsp_spi_stack_area, CONFIG_ETH_ADIN2111_BSP_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(bsp_gpio_stack_area, CONFIG_ETH_ADIN2111_BSP_THREAD_STACK_SIZE);
struct k_thread bsp_spi_thread_data;
struct k_thread bsp_gpio_thread_data;

#define RESET_GPIO DT_ALIAS(reset_gpio)
#if !DT_NODE_HAS_STATUS(RESET_GPIO, okay)
#error "Unsupported board: reset-gpio devicetree alias is not defined"
#endif

#define INT_GPIO DT_ALIAS(int_gpio)
#if !DT_NODE_HAS_STATUS(INT_GPIO, okay)
#error "Unsupported board: int-gpio devicetree alias is not defined"
#endif

static struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_GET_OR(RESET_GPIO, gpios, {0});
static struct gpio_dt_spec int_gpio = GPIO_DT_SPEC_GET_OR(INT_GPIO, gpios, {0});
struct spi_dt_spec adin_spi = SPI_DT_SPEC_GET(DT_NODELABEL(spi_adin2111), SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0);

K_SEM_DEFINE(pending_irq_sem, 0, 1);
K_SEM_DEFINE(gpio_sem, 0, 1);
K_SEM_DEFINE(spi_sem, 0, 1);

static void BSP_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void BSP_spi_thread(void);
static void BSP_gpio_thread(void);

static const struct gpio_dt_spec user5 = GPIO_DT_SPEC_GET(DT_ALIAS(user5), gpios);
static const struct gpio_dt_spec user6 = GPIO_DT_SPEC_GET(DT_ALIAS(user6), gpios);

static void BSP_gpio_callback(const struct device *dev,
                struct gpio_callback *cb,
                uint32_t pins) {
    k_sem_give(&gpio_sem);
}

/* Responds to GPIO interrupts */
static void BSP_gpio_thread(void) {
    k_thread_custom_data_set((void *) &user6);
    while(1) {
        k_sem_take(&gpio_sem, K_FOREVER);
        if (gpfIntCallback) {
            (*gpfIntCallback)(gpIntCBParam, 0, NULL);
        } else {
            /* No GPIO Callback function assigned */
        }
    }
}

/* Responds to SPI interrupts */
static void BSP_spi_thread(void) {
    k_thread_custom_data_set((void *) &user5);
    while(1) {
        k_sem_take(&spi_sem, K_FOREVER);
        if (gpfSpiCallback) {
            (*gpfSpiCallback)(gpSpiCBParam, 0, NULL);
        } else {
            /* No SPI Callback function assigned */
        }
    }
}

void BSP_delayMs(uint32_t delay) {
    k_sleep(K_MSEC(delay)); 
}

void BSP_HWReset(bool set) {
    gpio_port_clear_bits(reset_gpio.port, BIT(reset_gpio.pin));
    BSP_delayMs(RESET_DELAY);
    gpio_port_set_bits(reset_gpio.port, BIT(reset_gpio.pin));
    BSP_delayMs(AFTER_RESET_DELAY);
}

void BSP_INT_N_DisableIRQ(void) {    
    gpio_pin_interrupt_configure(int_gpio.port, int_gpio.pin, (GPIO_INT_DISABLE | 
                                                               GPIO_PULL_UP));
    return;
}

void BSP_INT_N_EnableIRQ(void) {
    gpio_pin_interrupt_configure(int_gpio.port, int_gpio.pin, (GPIO_INT_EDGE_FALLING | 
                                                               GPIO_PULL_UP));
    return;
}

void BSP_INT_N_SetPendingIRQ(void) {
    k_sem_give(&gpio_sem);   
}

uint32_t BSP_spi2_write_and_read(uint8_t *pBufferTx, uint8_t *pBufferRx, uint32_t nbBytes, bool useDma) {
    int ret;
    const struct spi_buf tx_buf = {
            .buf = pBufferTx,
            .len = nbBytes,
      };
      const struct spi_buf_set tx = {
            .buffers = &tx_buf,
            .count = 1,
      };
    const struct spi_buf rx_buf = {
            .buf = pBufferRx,
            .len = nbBytes,
      };
      const struct spi_buf_set rx = {
            .buffers = &rx_buf,
            .count = 1,
      };

    ret = spi_transceive(adin_spi.bus, &adin_spi.config, &tx, &rx);
    if (ret != 0) {
        while(1);
    } else {
        k_sem_give(&spi_sem);
    }
    return ret;
}

uint32_t BSP_spi2_register_callback(ADI_CB const *pfCallback, void *const pCBParam) {
    gpfSpiCallback = (ADI_CB) pfCallback;
    gpSpiCBParam = pCBParam ;
    return 0;
}

uint32_t BSP_RegisterIRQCallback(ADI_CB const *intCallback, void * hDevice) {
    gpfIntCallback = (ADI_CB) intCallback;
    gpIntCBParam = hDevice;
    return 0;
}

uint32_t BSP_Init(void) {
    gpio_pin_interrupt_configure(int_gpio.port, int_gpio.pin, (GPIO_INT_EDGE_FALLING | 
                                                               GPIO_PULL_UP));
    
    gpio_init_callback(&gpio_cb, BSP_gpio_callback, BIT(int_gpio.pin)); 
    if (gpio_add_callback(int_gpio.port, &gpio_cb)) {
        return EINVAL;
    }

    k_thread_create(&bsp_spi_thread_data, bsp_spi_stack_area,
                    K_THREAD_STACK_SIZEOF(bsp_spi_stack_area),
                    (k_thread_entry_t)BSP_spi_thread,
                    NULL, NULL, NULL,
                    K_PRIO_COOP(CONFIG_ETH_ADIN2111_BSP_THREAD_PRIO),
                    0, K_NO_WAIT);
    k_thread_name_set(&bsp_spi_thread_data, "adin2111_bsp_spi_thread");

    k_thread_create(&bsp_gpio_thread_data, bsp_gpio_stack_area,
                    K_THREAD_STACK_SIZEOF(bsp_gpio_stack_area),
                    (k_thread_entry_t)BSP_gpio_thread,
                    NULL, NULL, NULL,
                    K_PRIO_COOP(CONFIG_ETH_ADIN2111_BSP_THREAD_PRIO),
                    0, K_NO_WAIT);
    k_thread_name_set(&bsp_gpio_thread_data, "adin2111_bsp_gpio_thread");
    return 0;   
}

