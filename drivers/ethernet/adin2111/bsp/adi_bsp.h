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

#ifndef BOARDSUPPORT_H
#define BOARDSUPPORT_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

typedef void (* ADI_CB) (  /*!< Callback function pointer */
    void      *pCBParam,         /*!< Client supplied callback param */
    uint32_t   Event,            /*!< Event ID specific to the Driver/Service */
    void      *pArg);            /*!< Pointer to the event specific argument */

/*Functions prototypes*/
uint32_t BSP_RegisterIRQCallback (ADI_CB const *intCallback, void * hDevice);
uint32_t BSP_spi2_write_and_read (uint8_t *pBufferTx, uint8_t *pBufferRx, uint32_t nbBytes, bool useDma);
uint32_t BSP_spi2_register_callback (ADI_CB const *pfCallback, void *const pCBParam);

void BSP_HWReset(bool set);
void BSP_delayMs(uint32_t delay);
void BSP_INT_N_DisableIRQ(void);
void BSP_INT_N_EnableIRQ(void);
void BSP_INT_N_SetPendingIRQ(void);

uint32_t BSP_Init(void);
#endif /* BOARDSUPPORT_H */
