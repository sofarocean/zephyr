/*
 * Copyright (c) 2020 Linaro Ltd
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_BRISTLEMOUTH_BM_H_
#define ZEPHYR_DRIVERS_BRISTLEMOUTH_BM_H_

#include <zephyr/types.h>
#include <zephyr/random/rand32.h>

static inline void gen_random_mac(uint8_t *mac_addr, uint8_t b0, uint8_t b1, uint8_t b2)
{
	uint32_t entropy;

	entropy = sys_rand32_get();

	mac_addr[0] = b0;
	mac_addr[1] = b1;
	mac_addr[2] = b2;

	/* Set MAC address locally administered, unicast (LAA) */
	mac_addr[0] |= 0x02;

	mac_addr[3] = (entropy >> 16) & 0xff;
	mac_addr[4] = (entropy >>  8) & 0xff;
	mac_addr[5] = (entropy >>  0) & 0xff;
}

#endif /* ZEPHYR_DRIVERS_BRISTLEMOUTH_BM_H_ */
