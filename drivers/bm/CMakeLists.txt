# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_link_libraries(MCUBOOT_BOOTUTIL)
zephyr_library_sources_ifdef(CONFIG_BM_DFU bm_dfu.c)
zephyr_library_sources_ifdef(CONFIG_BM_COMMON bm_common.c)
