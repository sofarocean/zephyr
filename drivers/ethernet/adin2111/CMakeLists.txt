# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_ETH_ADIN2111	adi_mac.c)
zephyr_library_sources_ifdef(CONFIG_ETH_ADIN2111	adi_phy.c)
zephyr_library_sources_ifndef(CONFIG_SPI_OA_EN	    adi_spi_generic.c)
zephyr_library_sources_ifdef(CONFIG_SPI_OA_EN	    adi_spi_oa.c)
zephyr_library_sources_ifdef(CONFIG_ETH_ADIN2111	adin2111.c)
zephyr_library_sources_ifdef(CONFIG_TRACING_USER	tracing_user.c)

add_subdirectory(bsp)
add_subdirectory(hal)
