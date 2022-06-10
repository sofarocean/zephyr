/** @file
 * @brief Bristlemouth Common 
 *
 * A Bristlemouth Common API
 */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-Licens
 */

#include <stdint.h>
#include <stdlib.h>
#include <drivers/bm/bm_common.h>

void bm_util_linear_memcpy(uint8_t* dest, uint8_t* src, size_t n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        dest[i] = src[i];
    }
}