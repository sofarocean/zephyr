# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BM_STM32_HAL	bm_stm32_hal.c)
zephyr_library_sources(manchester.c)

zephyr_library_include_directories( PRIVATE ${ZEPHYR_BASE}/subsys/net/ip )

