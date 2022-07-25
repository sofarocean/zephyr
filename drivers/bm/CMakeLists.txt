# Copyright (c) 2021-2022 Sofar Ocean Technologies
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_BM)
    zephyr_library()

    # Common APIs
    zephyr_library_sources( bm_common.c )

    # Serial Interface
    zephyr_library_sources_ifdef(CONFIG_BM_SERIAL bm_serial.c)
    zephyr_library_sources_ifdef(CONFIG_BM_DFU_SERIAL bm_dfu_serial.c)

    # DFU Subsystem
    zephyr_library_link_libraries_ifdef(CONFIG_BM_DFU MCUBOOT_BOOTUTIL)
    zephyr_library_sources_ifdef(CONFIG_BM_DFU bm_dfu_core.c)
    zephyr_library_sources_ifdef(CONFIG_BM_DFU_CLIENT bm_dfu_client.c)
    zephyr_library_sources_ifdef(CONFIG_BM_DFU_HOST bm_dfu_host.c)
endif()