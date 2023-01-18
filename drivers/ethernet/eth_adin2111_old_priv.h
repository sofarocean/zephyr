/*
 * Copyright (c) 2020 DENX Software Engineering GmbH
 *               Lukasz Majewski <lukma@denx.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#ifndef __ETH_ADIN2111_PRIV_H__
#define __ETH_ADIN2111_PRIV_H__

/* ====================================================================================================
        MAC Module Register Address Offset Definitions
   ==================================================================================================== */
#define ADIN2111_IDVER                                                              0x00
#define ADIN2111_PHYID                                                              0x01
#define ADIN2111_CAPABILITY                                                         0x02
#define ADIN2111_RESET                                                              0x03
#define ADIN2111_CONFIG0                                                            0x04
#define ADIN2111_CONFIG2                                                            0x06
#define ADIN2111_STATUS0                                                            0x08
#define ADIN2111_STATUS1                                                            0x09
#define ADIN2111_BUFSTS                                                             0x0B
#define ADIN2111_IMASK0                                                             0x0C
#define ADIN2111_IMASK1                                                             0x0D
#define ADIN2111_TTSCAH                                                             0x10
#define ADIN2111_TTSCAL                                                             0x11
#define ADIN2111_TTSCBH                                                             0x12
#define ADIN2111_TTSCBL                                                             0x13
#define ADIN2111_TTSCCH                                                             0x14
#define ADIN2111_TTSCCL                                                             0x15
#define ADIN2111_MDIOACCn_START                                                     0x20 /* MDIO Access Registers  - 0x20 to 0x27 (by 1) */
#define ADIN2111_TX_FSIZE                                                           0x30
#define ADIN2111_TX                                                                 0x31
#define ADIN2111_TX_SPACE                                                           0x32
#define ADIN2111_TX_THRESH                                                          0x34
#define ADIN2111_TX_PRI                                                             0x35
#define ADIN2111_FIFO_CLR                                                           0x36
#define ADIN2111_SCRATCHn_START                                                     0x37 /* Scratch Registers  - 0x37 to 0x3A (by 1) */
#define ADIN2111_MAC_RST_STATUS                                                     0x3B
#define ADIN2111_SOFT_RST                                                           0x3C
#define ADIN2111_SPI_INJ_ERR                                                        0x3D
#define ADIN2111_FIFO_SIZE                                                          0x3E
#define ADIN2111_TFC                                                                0x3F
#define ADIN2111_TXSIZE                                                             0x40
#define ADIN2111_HTX_OVF_FRM_CNT                                                    0x41
#define ADIN2111_MECC_ERR_ADDR                                                      0x42
#define ADIN2111_CECC_ERRn_START                                                    0x43 /* Corrected ECC Error Counters  - 0x43 to 0x49 (by 1) */
#define ADIN2111_ADDR_FILT_UPRn_START                                               0x50 /* MAC Address Rule and DA Filter Upper 16 Bits Registers - 0x50 to 0x6E (by 2) */
#define ADIN2111_ADDR_FILT_LWRn_START                                               0x51 /* MAC Address DA Filter Lower 32 Bits Registers - 0x51 to 0x6F (by 2) */
#define ADIN2111_ADDR_MSK_UPRn_START                                                0x70 /* Upper 16 bits of MAC Address Mask - 0x70 to 0x72 (by 2) */
#define ADIN2111_ADDR_MSK_LWRn_START                                                0x71 /* Lower 32 bits of MAC Address Mask - 0x71 to 0x73 (by 2) */
#define ADIN2111_TS_ADDEND                                                          0x80
#define ADIN2111_TS_1SEC_CMP                                                        0x81
#define ADIN2111_TS_SEC_CNT                                                         0x82
#define ADIN2111_TS_NS_CNT                                                          0x83
#define ADIN2111_TS_CFG                                                             0x84
#define ADIN2111_TS_TIMER_HI                                                        0x85
#define ADIN2111_TS_TIMER_LO                                                        0x86
#define ADIN2111_TS_TIMER_QE_CORR                                                   0x87
#define ADIN2111_TS_TIMER_START                                                     0x88
#define ADIN2111_TS_EXT_CAPT0                                                       0x89
#define ADIN2111_TS_EXT_CAPT1                                                       0x8A
#define ADIN2111_TS_FREECNT_CAPT                                                    0x8B
#define ADIN2111_P1_RX_FSIZE                                                        0x90
#define ADIN2111_P1_RX                                                              0x91
#define ADIN2111_P1_RX_FRM_CNT                                                      0xA0
#define ADIN2111_P1_RX_BCAST_CNT                                                    0xA1
#define ADIN2111_P1_RX_MCAST_CNT                                                    0xA2
#define ADIN2111_P1_RX_UCAST_CNT                                                    0xA3
#define ADIN2111_P1_RX_CRC_ERR_CNT                                                  0xA4
#define ADIN2111_P1_RX_ALGN_ERR_CNT                                                 0xA5
#define ADIN2111_P1_RX_LS_ERR_CNT                                                   0xA6
#define ADIN2111_P1_RX_PHY_ERR_CNT                                                  0xA7
#define ADIN2111_P1_TX_FRM_CNT                                                      0xA8
#define ADIN2111_P1_TX_BCAST_CNT                                                    0xA9
#define ADIN2111_P1_TX_MCAST_CNT                                                    0xAA
#define ADIN2111_P1_TX_UCAST_CNT                                                    0xAB
#define ADIN2111_P1_RX_DROP_FULL_CNT                                                0xAC
#define ADIN2111_P1_RX_DROP_FILT_CNT                                                0xAD
#define ADIN2111_P1_RX_IFG_ERR_CNT                                                  0xAE
#define ADIN2111_P1_TX_IFG                                                          0xB0
#define ADIN2111_P1_LOOP                                                            0xB3
#define ADIN2111_P1_RX_CRC_EN                                                       0xB4
#define ADIN2111_P1_RX_IFG                                                          0xB5
#define ADIN2111_P1_RX_MAX_LEN                                                      0xB6
#define ADIN2111_P1_RX_MIN_LEN                                                      0xB7
#define ADIN2111_P1_LO_RFC                                                          0xB8
#define ADIN2111_P1_HI_RFC                                                          0xB9
#define ADIN2111_P1_LO_RXSIZE                                                       0xBA
#define ADIN2111_P1_HI_RXSIZE                                                       0xBB
#define ADIN2111_P1TOP2_TXSIZE                                                      0xBC
#define ADIN2111_P2_RX_FSIZE                                                        0xC0
#define ADIN2111_P2_RX                                                              0xC1
#define ADIN2111_P2_RX_FRM_CNT                                                      0xD0
#define ADIN2111_P2_RX_BCAST_CNT                                                    0xD1
#define ADIN2111_P2_RX_MCAST_CNT                                                    0xD2
#define ADIN2111_P2_RX_UCAST_CNT                                                    0xD3
#define ADIN2111_P2_RX_CRC_ERR_CNT                                                  0xD4
#define ADIN2111_P2_RX_ALGN_ERR_CNT                                                 0xD5
#define ADIN2111_P2_RX_LS_ERR_CNT                                                   0xD6
#define ADIN2111_P2_RX_PHY_ERR_CNT                                                  0xD7
#define ADIN2111_P2_TX_FRM_CNT                                                      0xD8
#define ADIN2111_P2_TX_BCAST_CNT                                                    0xD9
#define ADIN2111_P2_TX_MCAST_CNT                                                    0xDA
#define ADIN2111_P2_TX_UCAST_CNT                                                    0xDB
#define ADIN2111_P2_RX_DROP_FULL_CNT                                                0xDC
#define ADIN2111_P2_RX_DROP_FILT_CNT                                                0xDD
#define ADIN2111_P2_RX_IFG_ERR_CNT                                                  0xDE
#define ADIN2111_P2_TX_IFG                                                          0xE0
#define ADIN2111_P2_LOOP                                                            0xE3
#define ADIN2111_P2_RX_CRC_EN                                                       0xE4
#define ADIN2111_P2_RX_IFG                                                          0xE5
#define ADIN2111_P2_RX_MAX_LEN                                                      0xE6
#define ADIN2111_P2_RX_MIN_LEN                                                      0xE7
#define ADIN2111_P2_LO_RFC                                                          0xE8
#define ADIN2111_P2_HI_RFC                                                          0xE9
#define ADIN2111_P2_LO_RXSIZE                                                       0xEA
#define ADIN2111_P2_HI_RXSIZE                                                       0xEB
#define ADIN2111_P2TOP1_TXSIZE                                                      0xEC
#define ADIN2111_P2_TTSCAH                                                          0xF0
#define ADIN2111_P2_TTSCAL                                                          0xF1
#define ADIN2111_P2_TTSCBH                                                          0xF2
#define ADIN2111_P2_TTSCBL                                                          0xF3
#define ADIN2111_P2_TTSCCH                                                          0xF4
#define ADIN2111_P2_TTSCCL                                                          0xF5

/* ====================================================================================================
        MAC Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_MAC_IDVER                                                            0x10
#define RSTVAL_MAC_PHYID                                                            0x283BCA1
#define RSTVAL_MAC_CAPABILITY                                                       0x6C3
#define RSTVAL_MAC_RESET                                                            0x0
#define RSTVAL_MAC_CONFIG0                                                          0x6
#define RSTVAL_MAC_CONFIG2                                                          0x800
#define RSTVAL_MAC_STATUS0                                                          0x40
#define RSTVAL_MAC_STATUS1                                                          0x0
#define RSTVAL_MAC_BUFSTS                                                           0x3B00
#define RSTVAL_MAC_IMASK0                                                           0x1FBF
#define RSTVAL_MAC_IMASK1                                                           0x43FA1F1A
#define RSTVAL_MAC_TTSCAH                                                           0x0
#define RSTVAL_MAC_TTSCAL                                                           0x0
#define RSTVAL_MAC_TTSCBH                                                           0x0
#define RSTVAL_MAC_TTSCBL                                                           0x0
#define RSTVAL_MAC_TTSCCH                                                           0x0
#define RSTVAL_MAC_TTSCCL                                                           0x0
#define RSTVAL_MAC_MDIOACC_0_                                                       0x8C000000
#define RSTVAL_MAC_MDIOACC_1_                                                       0x8C000000
#define RSTVAL_MAC_MDIOACC_2_                                                       0x8C000000
#define RSTVAL_MAC_MDIOACC_3_                                                       0x8C000000
#define RSTVAL_MAC_MDIOACC_4_                                                       0x8C000000
#define RSTVAL_MAC_MDIOACC_5_                                                       0x8C000000
#define RSTVAL_MAC_MDIOACC_6_                                                       0x8C000000
#define RSTVAL_MAC_MDIOACC_7_                                                       0x8C000000
#define RSTVAL_MAC_TX_FSIZE                                                         0x0
#define RSTVAL_MAC_TX                                                               0x0
#define RSTVAL_MAC_TX_SPACE                                                         0x7FF
#define RSTVAL_MAC_TX_THRESH                                                        0x41
#define RSTVAL_MAC_TX_PRI                                                           0x0
#define RSTVAL_MAC_FIFO_CLR                                                         0x0
#define RSTVAL_MAC_SCRATCH_0_                                                       0x0
#define RSTVAL_MAC_SCRATCH_1_                                                       0x0
#define RSTVAL_MAC_SCRATCH_2_                                                       0x0
#define RSTVAL_MAC_SCRATCH_3_                                                       0x0
#define RSTVAL_MAC_MAC_RST_STATUS                                                   0x3
#define RSTVAL_MAC_SOFT_RST                                                         0x0
#define RSTVAL_MAC_SPI_INJ_ERR                                                      0x0
#define RSTVAL_MAC_FIFO_SIZE                                                        0x02222222
#define RSTVAL_MAC_TFC                                                              0x0
#define RSTVAL_MAC_TXSIZE                                                           0x0
#define RSTVAL_MAC_HTX_OVF_FRM_CNT                                                  0x0
#define RSTVAL_MAC_MECC_ERR_ADDR                                                    0x0
#define RSTVAL_MAC_CECC_ERR_0_                                                      0x0
#define RSTVAL_MAC_CECC_ERR_1_                                                      0x0
#define RSTVAL_MAC_CECC_ERR_2_                                                      0x0
#define RSTVAL_MAC_CECC_ERR_3_                                                      0x0
#define RSTVAL_MAC_CECC_ERR_4_                                                      0x0
#define RSTVAL_MAC_CECC_ERR_5_                                                      0x0
#define RSTVAL_MAC_CECC_ERR_6_                                                      0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_0_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_1_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_2_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_3_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_4_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_5_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_6_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_7_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_8_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_9_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_10_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_11_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_12_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_13_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_14_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_UPR_15_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_0_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_1_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_2_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_3_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_4_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_5_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_6_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_7_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_8_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_9_                                                 0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_10_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_11_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_12_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_13_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_14_                                                0x0
#define RSTVAL_MAC_ADDR_FILT_LWR_15_                                                0x0
#define RSTVAL_MAC_ADDR_MSK_UPR_0_                                                  0xFFFF
#define RSTVAL_MAC_ADDR_MSK_UPR_1_                                                  0xFFFF
#define RSTVAL_MAC_ADDR_MSK_LWR_0_                                                  0xFFFFFFFF
#define RSTVAL_MAC_ADDR_MSK_LWR_1_                                                  0xFFFFFFFF
#define RSTVAL_MAC_TS_ADDEND                                                        0x85555555
#define RSTVAL_MAC_TS_1SEC_CMP                                                      0x3B9ACA00
#define RSTVAL_MAC_TS_SEC_CNT                                                       0x0
#define RSTVAL_MAC_TS_NS_CNT                                                        0x0
#define RSTVAL_MAC_TS_CFG                                                           0x0
#define RSTVAL_MAC_TS_TIMER_HI                                                      0x0
#define RSTVAL_MAC_TS_TIMER_LO                                                      0x0
#define RSTVAL_MAC_TS_TIMER_QE_CORR                                                 0x0
#define RSTVAL_MAC_TS_TIMER_START                                                   0x0
#define RSTVAL_MAC_TS_EXT_CAPT0                                                     0x0
#define RSTVAL_MAC_TS_EXT_CAPT1                                                     0x0
#define RSTVAL_MAC_TS_FREECNT_CAPT                                                  0x0
#define RSTVAL_MAC_P1_RX_FSIZE                                                      0x0
#define RSTVAL_MAC_P1_RX                                                            0x0
#define RSTVAL_MAC_P1_RX_FRM_CNT                                                    0x0
#define RSTVAL_MAC_P1_RX_BCAST_CNT                                                  0x0
#define RSTVAL_MAC_P1_RX_MCAST_CNT                                                  0x0
#define RSTVAL_MAC_P1_RX_UCAST_CNT                                                  0x0
#define RSTVAL_MAC_P1_RX_CRC_ERR_CNT                                                0x0
#define RSTVAL_MAC_P1_RX_ALGN_ERR_CNT                                               0x0
#define RSTVAL_MAC_P1_RX_LS_ERR_CNT                                                 0x0
#define RSTVAL_MAC_P1_RX_PHY_ERR_CNT                                                0x0
#define RSTVAL_MAC_P1_TX_FRM_CNT                                                    0x0
#define RSTVAL_MAC_P1_TX_BCAST_CNT                                                  0x0
#define RSTVAL_MAC_P1_TX_MCAST_CNT                                                  0x0
#define RSTVAL_MAC_P1_TX_UCAST_CNT                                                  0x0
#define RSTVAL_MAC_P1_RX_DROP_FULL_CNT                                              0x0
#define RSTVAL_MAC_P1_RX_DROP_FILT_CNT                                              0x0
#define RSTVAL_MAC_P1_RX_IFG_ERR_CNT                                                0x0
#define RSTVAL_MAC_P1_TX_IFG                                                        0xB
#define RSTVAL_MAC_P1_LOOP                                                          0x0
#define RSTVAL_MAC_P1_RX_CRC_EN                                                     0x1
#define RSTVAL_MAC_P1_RX_IFG                                                        0xA
#define RSTVAL_MAC_P1_RX_MAX_LEN                                                    0x618
#define RSTVAL_MAC_P1_RX_MIN_LEN                                                    0x40
#define RSTVAL_MAC_P1_LO_RFC                                                        0x0
#define RSTVAL_MAC_P1_HI_RFC                                                        0x0
#define RSTVAL_MAC_P1_LO_RXSIZE                                                     0x0
#define RSTVAL_MAC_P1_HI_RXSIZE                                                     0x0
#define RSTVAL_MAC_P1TOP2_TXSIZE                                                    0x0
#define RSTVAL_MAC_P2_RX_FSIZE                                                      0x0
#define RSTVAL_MAC_P2_RX                                                            0x0
#define RSTVAL_MAC_P2_RX_FRM_CNT                                                    0x0
#define RSTVAL_MAC_P2_RX_BCAST_CNT                                                  0x0
#define RSTVAL_MAC_P2_RX_MCAST_CNT                                                  0x0
#define RSTVAL_MAC_P2_RX_UCAST_CNT                                                  0x0
#define RSTVAL_MAC_P2_RX_CRC_ERR_CNT                                                0x0
#define RSTVAL_MAC_P2_RX_ALGN_ERR_CNT                                               0x0
#define RSTVAL_MAC_P2_RX_LS_ERR_CNT                                                 0x0
#define RSTVAL_MAC_P2_RX_PHY_ERR_CNT                                                0x0
#define RSTVAL_MAC_P2_TX_FRM_CNT                                                    0x0
#define RSTVAL_MAC_P2_TX_BCAST_CNT                                                  0x0
#define RSTVAL_MAC_P2_TX_MCAST_CNT                                                  0x0
#define RSTVAL_MAC_P2_TX_UCAST_CNT                                                  0x0
#define RSTVAL_MAC_P2_RX_DROP_FULL_CNT                                              0x0
#define RSTVAL_MAC_P2_RX_DROP_FILL_CNT                                              0x0
#define RSTVAL_MAC_P2_RX_IFG_ERR_CNT                                                0x0
#define RSTVAL_MAC_P2_TX_IFG                                                        0x0
#define RSTVAL_MAC_P2_LOOP                                                          0x0
#define RSTVAL_MAC_P2_RX_CRC_EN                                                     0x1
#define RSTVAL_MAC_P2_RX_IFG                                                        0xA
#define RSTVAL_MAC_P2_RX_MAX_LEN                                                    0x618
#define RSTVAL_MAC_P2_RX_MIN_LEN                                                    0x40
#define RSTVAL_MAC_P2_LO_RFC                                                        0x00
#define RSTVAL_MAC_P2_HI_RFC                                                        0x00
#define RSTVAL_MAC_P2_LO_RXSIZE                                                     0x00
#define RSTVAL_MAC_P2_HI_RXSIZE                                                     0x00
#define RSTVAL_MAC_P2TOP1_TXSIZE                                                    0x00
#define RSTVAL_MAC_P2_TTSCAH                                                        0x00
#define RSTVAL_MAC_P2_TTSCAL                                                        0x00
#define RSTVAL_MAC_P2_TTSCBL                                                        0x00
#define RSTVAL_MAC_P2_TTSCCH                                                        0x00
#define RSTVAL_MAC_P2_TTSCCL                                                        0x00

/* ----------------------------------------------------------------------------------------------------
          IDVER
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_IDVER_MINVER                                                       0 /* OA Minor Version. */
#define BITL_MAC_IDVER_MINVER                                                       4 /* OA Minor Version. */
#define BITM_MAC_IDVER_MINVER                                                       0x0000000F /* OA Minor Version. */
#define BITP_MAC_IDVER_MAJVER                                                       4 /* OA Major Version. */
#define BITM_MAC_IDVER_MAJVER                                                       0x000000F0 /* OA Major Version. */
#define BITL_MAC_IDVER_MAJVER                                                       4 /* OA Major Version. */

/* ----------------------------------------------------------------------------------------------------
          PHYID
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_PHYID_REVISION                                                     0 /* Manufacturer’s Revision Number. */
#define BITL_MAC_PHYID_REVISION                                                     4 /* Manufacturer’s Revision Number. */
#define BITM_MAC_PHYID_REVISION                                                     0x0000000F /* Manufacturer’s Revision Number. */
#define BITP_MAC_PHYID_MODEL                                                        4 /* Manufacturer’s Model Number. */
#define BITL_MAC_PHYID_MODEL                                                        6 /* Manufacturer’s Model Number. */
#define BITM_MAC_PHYID_MODEL                                                        0x000003F0 /* Manufacturer’s Model Number. */
#define BITP_MAC_PHYID_OUI                                                          10 /* Organizationally Unique Identifier (Bits 2:23). */
#define BITL_MAC_PHYID_OUI                                                          22 /* Organizationally Unique Identifier (Bits 2:23). */
#define BITM_MAC_PHYID_OUI                                                          0xFFFFFC00 /* Organizationally Unique Identifier (Bits 2:23). */

/* ----------------------------------------------------------------------------------------------------
          CAPABILITY
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_CAPABILITY_MINCPS                                                  0 /* Minimum Supported Chunk Payload Size. */
#define BITL_MAC_CAPABILITY_MINCPS                                                  3 /* Minimum Supported Chunk Payload Size. */
#define BITM_MAC_CAPABILITY_MINCPS                                                  0x00000007 /* Minimum Supported Chunk Payload Size. */
#define BITP_MAC_CAPABILITY_SEQC                                                    4 /* TX Data Chunk Sequence and Retry Capability. */
#define BITL_MAC_CAPABILITY_SEQC                                                    1 /* TX Data Chunk Sequence and Retry Capability. */
#define BITM_MAC_CAPABILITY_SEQC                                                    0x00000010 /* TX Data Chunk Sequence and Retry Capability. */
#define BITP_MAC_CAPABILITY_AIDC                                                    5 /* Address Increment Disable Capability. */
#define BITL_MAC_CAPABILITY_AIDC                                                    1 /* Address Increment Disable Capability. */
#define BITM_MAC_CAPABILITY_AIDC                                                    0x00000020 /* Address Increment Disable Capability. */
#define BITP_MAC_CAPABILITY_FTSC                                                    6 /* Frame Timestamp Capability. */
#define BITL_MAC_CAPABILITY_FTSC                                                    1 /* Frame Timestamp Capability. */
#define BITM_MAC_CAPABILITY_FTSC                                                    0x00000040 /* Frame Timestamp Capability. */
#define BITP_MAC_CAPABILITY_CTC                                                     7 /* Cut-Through Capability. */
#define BITL_MAC_CAPABILITY_CTC                                                     1 /* Cut-Through Capability. */
#define BITM_MAC_CAPABILITY_CTC                                                     0x00000080 /* Cut-Through Capability. */
#define BITP_MAC_CAPABILITY_DPRAC                                                   8 /* Direct PHY Register Access Capability. */
#define BITL_MAC_CAPABILITY_DPRAC                                                   1 /* Direct PHY Register Access Capability. */
#define BITM_MAC_CAPABILITY_DPRAC                                                   0x00000100 /* Direct PHY Register Access Capability. */
#define BITP_MAC_CAPABILITY_IPRAC                                                   9 /* Indirect PHY Register Access Capability. */
#define BITL_MAC_CAPABILITY_IPRAC                                                   1 /* Indirect PHY Register Access Capability. */
#define BITM_MAC_CAPABILITY_IPRAC                                                   0x00000200 /* Indirect PHY Register Access Capability. */
#define BITP_MAC_CAPABILITY_TXFCSVC                                                 10 /* Transmit Frame Check Sequence Validation Capability. */
#define BITL_MAC_CAPABILITY_TXFCSVC                                                 1 /* Transmit Frame Check Sequence Validation Capability. */
#define BITM_MAC_CAPABILITY_TXFCSVC                                                 0x00000400 /* Transmit Frame Check Sequence Validation Capability. */

#define ENUM_MAC_CAPABILITY_TXFCSVC_TXFCSVC_0                                       0x00000000 /* Transmit FCS Validation is Not Supported. */
#define ENUM_MAC_CAPABILITY_TXFCSVC_TXFCSVC_1                                       0x00000001 /* Transmit FCS Validation is Supported. */
#define ENUM_MAC_CAPABILITY_IPRAC_IPRAC_0                                           0x00000000 /* PHY Registers are Not Indirectly Accessible. */
#define ENUM_MAC_CAPABILITY_IPRAC_IPRAC_1                                           0x00000001 /* PHY Registers are Indirectly Accessible. */
#define ENUM_MAC_CAPABILITY_DPRAC_DPRAC_0                                           0x00000000 /* PHY Registers are Not Directly Accessible. */
#define ENUM_MAC_CAPABILITY_DPRAC_DPRAC_1                                           0x00000001 /* PHY Registers are Directly Accessible. */
#define ENUM_MAC_CAPABILITY_CTC_CTC_0                                               0x00000000 /* Cut-Through Not Supported. */
#define ENUM_MAC_CAPABILITY_CTC_CTC_1                                               0x00000001 /* Cut-Through Supported. */
#define ENUM_MAC_CAPABILITY_FTSC_FTSC_1                                             0x00000001 /* IEEE 1588 Timestamp Capture on Frame Tx/Rx is Supported. */
#define ENUM_MAC_CAPABILITY_FTSC_FTSC_0                                             0x00000000 /* IEEE 1588 Timestamp Capture on Frame Tx/Rx is Not Supported. */
#define ENUM_MAC_CAPABILITY_SEQC_SEQC_1                                             0x00000001 /* TX Data Chunk Sequence and Retry is Supported. */
#define ENUM_MAC_CAPABILITY_SEQC_SEQC_0                                             0x00000000 /* TX Data Chunk Sequence and Retry is Not Supported. */
#define ENUM_MAC_CAPABILITY_MINCPS_CPS64                                            0x00000006 /* Minimum Supported Chunk Payload Size is 64 Bytes. */
#define ENUM_MAC_CAPABILITY_MINCPS_CPS32                                            0x00000005 /* Minimum Supported Chunk Payload Size is 32 Bytes. */
#define ENUM_MAC_CAPABILITY_MINCPS_CPS16                                            0x00000004 /* Minimum Supported Chunk Payload Size is 16 Bytes. */
#define ENUM_MAC_CAPABILITY_MINCPS_CPS8                                             0x00000003 /* Minimum Supported Chunk Payload Size is 8 Bytes. */

/* ----------------------------------------------------------------------------------------------------
          RESET                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_RESET_SWRESET                                                      0 /* Software Reset. */
#define BITL_MAC_RESET_SWRESET                                                      1 /* Software Reset. */
#define BITM_MAC_RESET_SWRESET                                                      0x00000001 /* Software Reset. */

/* ----------------------------------------------------------------------------------------------------
          CONFIG0
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_CONFIG0_CPS                                                        0 /* Chunk Payload Selector (N). */
#define BITL_MAC_CONFIG0_CPS                                                        3 /* Chunk Payload Selector (N). */
#define BITM_MAC_CONFIG0_CPS                                                        0x00000007 /* Chunk Payload Selector (N). */
#define BITP_MAC_CONFIG0_SEQE                                                       4 /* Enable TX Data Chunk Sequence and Retry. */
#define BITL_MAC_CONFIG0_SEQE                                                       1 /* Enable TX Data Chunk Sequence and Retry. */
#define BITM_MAC_CONFIG0_SEQE                                                       0x00000010 /* Enable TX Data Chunk Sequence and Retry. */
#define BITP_MAC_CONFIG0_PROTE                                                      5 /* Enable Control Data Read Write Protection. */
#define BITL_MAC_CONFIG0_PROTE                                                      1 /* Enable Control Data Read Write Protection. */
#define BITM_MAC_CONFIG0_PROTE                                                      0x00000020 /* Enable Control Data Read Write Protection. */
#define BITP_MAC_CONFIG0_FTSS                                                       6 /* Receive Frame Timestamp Select. */
#define BITL_MAC_CONFIG0_FTSS                                                       1 /* Receive Frame Timestamp Select. */
#define BITM_MAC_CONFIG0_FTSS                                                       0x00000040 /* Receive Frame Timestamp Select. */
#define BITP_MAC_CONFIG0_FTSE                                                       7 /* Frame Timestamp Enable. */
#define BITL_MAC_CONFIG0_FTSE                                                       1 /* Frame Timestamp Enable. */
#define BITM_MAC_CONFIG0_FTSE                                                       0x00000080 /* Frame Timestamp Enable. */
#define BITP_MAC_CONFIG0_RXCTE                                                      8 /* Receive Cut-Through Enable. */
#define BITL_MAC_CONFIG0_RXCTE                                                      1  /* Receive Cut-Through Enable. */
#define BITM_MAC_CONFIG0_RXCTE                                                      0x00000100 /* Receive Cut-Through Enable. */
#define BITP_MAC_CONFIG0_TXCTE                                                      9 /* Transmit Cut-Through Enable. */
#define BITL_MAC_CONFIG0_TXCTE                                                      1 /* Transmit Cut-Through Enable. */
#define BITM_MAC_CONFIG0_TXCTE                                                      0x00000200 /* Transmit Cut-Through Enable. */
#define BITP_MAC_CONFIG0_TXCTHRESH                                                  10 /* Transmit Credit Threshold. */
#define BITL_MAC_CONFIG0_TXCTHRESH                                                  2 /* Transmit Credit Threshold. */
#define BITM_MAC_CONFIG0_TXCTHRESH                                                  0x00000C00 /* Transmit Credit Threshold. */
#define BITP_MAC_CONFIG0_ZARFE                                                      12 /* Zero-Align Receive Frame Enable. */
#define BITL_MAC_CONFIG0_ZARFE                                                      1 /* Zero-Align Receive Frame Enable. */
#define BITM_MAC_CONFIG0_ZARFE                                                      0x00001000 /* Zero-Align Receive Frame Enable. */
#define BITP_MAC_CONFIG0_CSARFE                                                     13 /* CSn Align Receive Frame Enable. */
#define BITL_MAC_CONFIG0_CSARFE                                                     1 /* CSn Align Receive Frame Enable. */
#define BITM_MAC_CONFIG0_CSARFE                                                     0x00002000 /* CSn Align Receive Frame Enable. */
#define BITP_MAC_CONFIG0_TXFCSVE                                                    14 /* Transmit Frame Check Sequence Validation Enable. */
#define BITL_MAC_CONFIG0_TXFCSVE                                                    1 /* Transmit Frame Check Sequence Validation Enable. */
#define BITM_MAC_CONFIG0_TXFCSVE                                                    0x00004000 /* Transmit Frame Check Sequence Validation Enable. */
#define BITP_MAC_CONFIG0_SYNC                                                       15 /* Configuration Synchronization. */
#define BITL_MAC_CONFIG0_SYNC                                                       1 /* Configuration Synchronization. */
#define BITM_MAC_CONFIG0_SYNC                                                       0x00008000 /* Configuration Synchronization. */

#define ENUM_MAC_CONFIG0_SYNC_SYNC_0                                                0x00000000 /* The MACPHY Has Been Reset and is Not Configured. */
#define ENUM_MAC_CONFIG0_SYNC_SYNC_1                                                0x00000001 /* The MACPHY is Configured. */
#define ENUM_MAC_CONFIG0_TXCTHRESH_CREDIT_1                                         0x00000000 /* ≥ 1 Credit. */
#define ENUM_MAC_CONFIG0_TXCTHRESH_CREDIT_4                                         0x00000001 /* ≥ 4 Credits. */
#define ENUM_MAC_CONFIG0_TXCTHRESH_CREDIT_8                                         0x00000002 /* ≥ 8 Credits. */
#define ENUM_MAC_CONFIG0_TXCTHRESH_CREDIT_16                                        0x00000003 /* ≥ 16 Credits. */
#define ENUM_MAC_CONFIG0_FTSE_FTSE_0                                                0x00000000 /* Frame Receive/Transmit Timestamps are Disabled. */
#define ENUM_MAC_CONFIG0_FTSE_FTSE_1                                                0x00000001 /* Frame Receive/Transmit Timestamps are Enabled. */
#define ENUM_MAC_CONFIG0_FTSS_FTSS_0                                                0x00000000 /* 32-bit Timestamps. */
#define ENUM_MAC_CONFIG0_FTSS_FTSS_1                                                0x00000001 /* 64-bit Timestamps. */
#define ENUM_MAC_CONFIG0_CPS_CHUNK_8BYTE                                            0x00000003 /* No description provided */
#define ENUM_MAC_CONFIG0_CPS_CHUNK_16BYTE                                           0x00000004 /* No description provided */
#define ENUM_MAC_CONFIG0_CPS_CHUNK_32BYTE                                           0x00000005 /* No description provided */
#define ENUM_MAC_CONFIG0_CPS_CHUNK_64BYTE                                           0x00000006 /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CONFIG2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_CONFIG2_MSPEED                                                     0 /* SPI to MDIO Bridge MDC Clock Speed. */
#define BITL_MAC_CONFIG2_MSPEED                                                     2 /* SPI to MDIO Bridge MDC Clock Speed. */
#define BITM_MAC_CONFIG2_MSPEED                                                     0x00000003 /* SPI to MDIO Bridge MDC Clock Speed. */
#define BITP_MAC_CONFIG2_P1_FWD_UNK2HOST                                            2 /* Forward Frames Not Matching Any MAC Address to the Host. */
#define BITL_MAC_CONFIG2_P1_FWD_UNK2HOST                                            1 /* Forward Frames Not Matching Any MAC Address to the Host. */
#define BITM_MAC_CONFIG2_P1_FWD_UNK2HOST                                            0x00000004 /* Forward Frames Not Matching Any MAC Address to the Host. */
#define BITP_MAC_CONFIG2_P1_RCV_IFG_ERR_FRM                                         4 /* Admit Frames with IFG Errors on P1. */
#define BITL_MAC_CONFIG2_P1_RCV_IFG_ERR_FRM                                         1 /* Admit Frames with IFG Errors on P1. */
#define BITM_MAC_CONFIG2_P1_RCV_IFG_ERR_FRM                                         0x00000010 /* Admit Frames with IFG Errors on P1. */
#define BITP_MAC_CONFIG2_CRC_APPEND                                                 5 /* Enable CRC Append. */
#define BITL_MAC_CONFIG2_CRC_APPEND                                                 1 /* Enable CRC Append. */
#define BITM_MAC_CONFIG2_CRC_APPEND                                                 0x00000020 /* Enable CRC Append. */
#define BITP_MAC_CONFIG2_STATS_CLR_ON_RD                                            6 /* Statistics Clear on Reading. */
#define BITL_MAC_CONFIG2_STATS_CLR_ON_RD                                            1 /* Statistics Clear on Reading. */
#define BITM_MAC_CONFIG2_STATS_CLR_ON_RD                                            0x00000040 /* Statistics Clear on Reading. */
#define BITP_MAC_CONFIG2_SFD_DETECT_SRC                                             7 /* Determines If the SFD is Detected in the PHY or MAC. */
#define BITL_MAC_CONFIG2_SFD_DETECT_SRC                                             1 /* Determines If the SFD is Detected in the PHY or MAC. */
#define BITM_MAC_CONFIG2_SFD_DETECT_SRC                                             0x00000080 /* Determines If the SFD is Detected in the PHY or MAC. */
#define BITP_MAC_CONFIG2_TX_RDY_ON_EMPTY                                            8 /* Assert TX_RDY When the Tx FIFO is Empty. */
#define BITL_MAC_CONFIG2_TX_RDY_ON_EMPTY                                            1 /* Assert TX_RDY When the Tx FIFO is Empty. */
#define BITM_MAC_CONFIG2_TX_RDY_ON_EMPTY                                            0x00000100 /* Assert TX_RDY When the Tx FIFO is Empty. */
#define BITP_MAC_CONFIG2_PORT_CUT_THRU_EN                                           11
#define BITL_MAC_CONFIG2_PORT_CUT_THRU_EN                                           1
#define BITM_MAC_CONFIG2_PORT_CUT_THRU_EN                                           0x00000800
#define BITP_MAC_CONFIG2_P2_FWD_UNK2HOST                                            12
#define BITL_MAC_CONFIG2_P2_FWD_UNK2HOST                                            1
#define BITM_MAC_CONFIG2_P2_FWD_UNK2HOST                                            0x00001000
#define BITP_MAC_CONFIG2_P2_FWD_UNK2P2                                              13
#define BITL_MAC_CONFIG2_P2_FWD_UNK2P2                                              1
#define BITM_MAC_CONFIG2_P2_FWD_UNK2P2                                              0x00002000
#define BITP_MAC_CONFIG2_P2_FWD_UNK2P1                                              14
#define BITL_MAC_CONFIG2_P2_FWD_UNK2P1                                              1
#define BITM_MAC_CONFIG2_P2_FWD_UNK2P1                                              0x00004000
#define BITP_MAC_CONFIG2_RX_RD_ORDER                                                15
#define BITL_MAC_CONFIG2_RX_RD_ORDER                                                1
#define BITM_MAC_CONFIG2_RX_RD_ORDER                                                0x00008000

#define ENUM_MAC_CONFIG2_RX_RD_ORDER_FILL                                           0x00000000
#define ENUM_MAC_CONFIG2_RX_RD_ORDER_PING_PONG                                      0x00000001
#define ENUM_MAC_CONFIG2_SFD_DETECT_SRC_PHY                                         0x00000000 /* Select the SFD from the PHY. */
#define ENUM_MAC_CONFIG2_SFD_DETECT_SRC_MAC                                         0x00000001 /* Select the SFD from the MAC. */
#define ENUM_MAC_CONFIG2_STATS_CLR_ON_RD_HOLD_STAT                                  0x00000000 /* Statistic Counter is Not Cleared on Reading. */
#define ENUM_MAC_CONFIG2_STATS_CLR_ON_RD_CLR_STAT                                   0x00000001 /* Clear Statistics Counters on Reading. */
#define ENUM_MAC_CONFIG2_MSPEED_MHZ_2P5                                             0x00000000 /* 2.5 MHz. */
#define ENUM_MAC_CONFIG2_MSPEED_MHZ_4P166                                           0x00000001 /* 4.166 MHz. */

/* ----------------------------------------------------------------------------------------------------
          STATUS0
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_STATUS0_TXPE                                                       0 /* Transmit Protocol Error. */
#define BITL_MAC_STATUS0_TXPE                                                       1 /* Transmit Protocol Error. */
#define BITM_MAC_STATUS0_TXPE                                                       0x00000001 /* Transmit Protocol Error. */
#define BITP_MAC_STATUS0_TXBOE                                                      1 /* Host Tx FIFO Overflow. */
#define BITL_MAC_STATUS0_TXBOE                                                      1 /* Host Tx FIFO Overflow. */
#define BITM_MAC_STATUS0_TXBOE                                                      0x00000002 /* Host Tx FIFO Overflow. */
#define BITP_MAC_STATUS0_TXBUE                                                      2 /* Host Tx FIFO Underrun Error. */
#define BITL_MAC_STATUS0_TXBUE                                                      1 /* Host Tx FIFO Underrun Error. */
#define BITM_MAC_STATUS0_TXBUE                                                      0x00000004 /* Host Tx FIFO Underrun Error. */
#define BITP_MAC_STATUS0_RXBOE                                                      3 /* Receive Buffer Overflow Error. */
#define BITL_MAC_STATUS0_RXBOE                                                      1 /* Receive Buffer Overflow Error. */
#define BITM_MAC_STATUS0_RXBOE                                                      0x00000008U /* Receive Buffer Overflow Error. */
#define BITP_MAC_STATUS0_LOFE                                                       4 /* Loss of Frame Error. */
#define BITL_MAC_STATUS0_LOFE                                                       1 /* Loss of Frame Error. */
#define BITM_MAC_STATUS0_LOFE                                                       0x00000010 /* Loss of Frame Error. */
#define BITP_MAC_STATUS0_HDRE                                                       5 /* Header Error. */
#define BITL_MAC_STATUS0_HDRE                                                       1 /* Header Error. */
#define BITM_MAC_STATUS0_HDRE                                                       0x00000020 /* Header Error. */
#define BITP_MAC_STATUS0_RESETC                                                     6 /* Reset Complete. */
#define BITL_MAC_STATUS0_RESETC                                                     1 /* Reset Complete. */
#define BITM_MAC_STATUS0_RESETC                                                     0x00000040 /* Reset Complete. */
#define BITP_MAC_STATUS0_PHYINT                                                     7 /* PHY Interrupt for Port1. */
#define BITL_MAC_STATUS0_PHYINT                                                     1 /* PHY Interrupt for Port1. */
#define BITM_MAC_STATUS0_PHYINT                                                     0x00000080 /* PHY Interrupt for Port1. */
#define BITP_MAC_STATUS0_TTSCAA                                                     8 /* Transmit Timestamp Capture Available A. */
#define BITL_MAC_STATUS0_TTSCAA                                                     1 /* Transmit Timestamp Capture Available A. */
#define BITM_MAC_STATUS0_TTSCAA                                                     0x00000100 /* Transmit Timestamp Capture Available A. */
#define BITP_MAC_STATUS0_TTSCAB                                                     9 /* Transmit Timestamp Capture Available B. */
#define BITL_MAC_STATUS0_TTSCAB                                                     1 /* Transmit Timestamp Capture Available B. */
#define BITM_MAC_STATUS0_TTSCAB                                                     0x00000200 /* Transmit Timestamp Capture Available B. */
#define BITP_MAC_STATUS0_TTSCAC                                                     10 /* Transmit Timestamp Capture Available C. */
#define BITL_MAC_STATUS0_TTSCAC                                                     1 /* Transmit Timestamp Capture Available C. */
#define BITM_MAC_STATUS0_TTSCAC                                                     0x00000400 /* Transmit Timestamp Capture Available C. */
#define BITP_MAC_STATUS0_TXFCSE                                                     11 /* Transmit Frame Check Sequence Error. */
#define BITL_MAC_STATUS0_TXFCSE                                                     1 /* Transmit Frame Check Sequence Error. */
#define BITM_MAC_STATUS0_TXFCSE                                                     0x00000800 /* Transmit Frame Check Sequence Error. */
#define BITP_MAC_STATUS0_CDPE                                                       12 /* Control Data Protection Error. */
#define BITL_MAC_STATUS0_CDPE                                                       1 /* Control Data Protection Error. */
#define BITM_MAC_STATUS0_CDPE                                                       0x00001000 /* Control Data Protection Error. */

/* ----------------------------------------------------------------------------------------------------
          STATUS1 for ADIN2111
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_STATUS1_TX_RDY                                                     3 /* Tx Ready. */
#define BITL_MAC_STATUS1_TX_RDY                                                     1 /* Tx Ready. */
#define BITM_MAC_STATUS1_TX_RDY                                                     0x00000008 /* Tx Ready. */
#define BITP_MAC_STATUS1_P1_RX_RDY                                                  4 /* Port 1 Rx FIFO Contains Data. */
#define BITL_MAC_STATUS1_P1_RX_RDY                                                  1 /* Port 1 Rx FIFO Contains Data. */
#define BITM_MAC_STATUS1_P1_RX_RDY                                                  0x00000010 /* Port 1 Rx FIFO Contains Data. */
#define BITP_MAC_STATUS1_P1_RX_RDY_HI                                               5 /* Port1 Rx Ready High Priority. */
#define BITL_MAC_STATUS1_P1_RX_RDY_HI                                               1 /* Port1 Rx Ready High Priority. */
#define BITM_MAC_STATUS1_P1_RX_RDY_HI                                               0x00000020 /* Port1 Rx Ready High Priority. */
#define BITP_MAC_STATUS1_P1_RX_IFG_ERR                                              8 /* Rx MAC Inter Frame Gap Error. */
#define BITL_MAC_STATUS1_P1_RX_IFG_ERR                                              1 /* Rx MAC Inter Frame Gap Error. */
#define BITM_MAC_STATUS1_P1_RX_IFG_ERR                                              0x00000100 /* Rx MAC Inter Frame Gap Error. */
#define BITP_MAC_STATUS1_SPI_ERR                                                    10 /* Detected an Error on an SPI Transaction. */
#define BITL_MAC_STATUS1_SPI_ERR                                                    1 /* Detected an Error on an SPI Transaction. */
#define BITM_MAC_STATUS1_SPI_ERR                                                    0x00000400 /* Detected an Error on an SPI Transaction. */
#define BITP_MAC_STATUS1_RX_ECC_ERR                                                 11 /* ECC Error on Reading the Frame Size from an Rx FIFO. */
#define BITL_MAC_STATUS1_RX_ECC_ERR                                                 1 /* ECC Error on Reading the Frame Size from an Rx FIFO. */
#define BITM_MAC_STATUS1_RX_ECC_ERR                                                 0x00000800 /* ECC Error on Reading the Frame Size from an Rx FIFO. */
#define BITP_MAC_STATUS1_TX_ECC_ERR                                                 12 /* ECC Error on Reading the Frame Size from a Tx FIFO. */
#define BITL_MAC_STATUS1_TX_ECC_ERR                                                 1 /* ECC Error on Reading the Frame Size from a Tx FIFO. */
#define BITM_MAC_STATUS1_TX_ECC_ERR                                                 0x00001000 /* ECC Error on Reading the Frame Size from a Tx FIFO. */
#define BITP_MAC_STATUS1_P2_RX_RDY                                                  17
#define BITL_MAC_STATUS1_P2_RX_RDY                                                  1
#define BITM_MAC_STATUS1_P2_RX_RDY                                                  0x00020000
#define BITP_MAC_STATUS1_P2_RX_RDY_HI                                               18
#define BITL_MAC_STATUS1_P2_RX_RDY_HI                                               1
#define BITM_MAC_STATUS1_P2_RX_RDY_HI                                               0x00040000
#define BITP_MAC_STATUS1_P2_PHYINT                                                  19
#define BITL_MAC_STATUS1_P2_PHYINT                                                  1
#define BITM_MAC_STATUS1_P2_PHYINT                                                  0x00080000
#define BITP_MAC_STATUS1_P2_TTSCAA                                                  20
#define BITL_MAC_STATUS1_P2_TTSCAA                                                  1
#define BITM_MAC_STATUS1_P2_TTSCAA                                                  0x00100000
#define BITP_MAC_STATUS1_P2_TTSCAB                                                  21
#define BITL_MAC_STATUS1_P2_TTSCAB                                                  1
#define BITM_MAC_STATUS1_P2_TTSCAB                                                  0x00200000
#define BITP_MAC_STATUS1_P2_TTSCAC                                                  22
#define BITL_MAC_STATUS1_P2_TTSCAC                                                  1
#define BITM_MAC_STATUS1_P2_TTSCAC                                                  0x00400000
#define BITP_MAC_STATUS1_P2_RX_IFG_ERR                                              23
#define BITL_MAC_STATUS1_P2_RX_IFG_ERR                                              1
#define BITM_MAC_STATUS1_P2_RX_IFG_ERR                                              0x00800000
#define BITP_MAC_STATUS1_P2_TXFCSE                                                  24
#define BITL_MAC_STATUS1_P2_TXFCSE                                                  1
#define BITM_MAC_STATUS1_P2_TXFCSE                                                  0x01000000

/* ----------------------------------------------------------------------------------------------------
          BUFSTS
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_BUFSTS_RCA                                                         0 /* Receive Chunks Available. */
#define BITL_MAC_BUFSTS_RCA                                                         8 /* Receive Chunks Available. */
#define BITM_MAC_BUFSTS_RCA                                                         0x000000FF /* Receive Chunks Available. */
#define BITP_MAC_BUFSTS_TXC                                                         8 /* Transmit Credits Available. */
#define BITL_MAC_BUFSTS_TXC                                                         8 /* Transmit Credits Available. */
#define BITM_MAC_BUFSTS_TXC                                                         0x0000FF00 /* Transmit Credits Available. */

/* ----------------------------------------------------------------------------------------------------
          IMASK0
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_IMASK0_TXPEM                                                       0 /* Transmit Protocol Error Mask. */
#define BITL_MAC_IMASK0_TXPEM                                                       1 /* Transmit Protocol Error Mask. */
#define BITM_MAC_IMASK0_TXPEM                                                       0x00000001 /* Transmit Protocol Error Mask. */
#define BITP_MAC_IMASK0_TXBOEM                                                      1 /* Transmit Buffer Overflow Error Mask. */
#define BITL_MAC_IMASK0_TXBOEM                                                      1 /* Transmit Buffer Overflow Error Mask. */
#define BITM_MAC_IMASK0_TXBOEM                                                      0x00000002 /* Transmit Buffer Overflow Error Mask. */
#define BITP_MAC_IMASK0_TXBUEM                                                      2 /* Transmit Buffer Underflow Error Mask. */
#define BITL_MAC_IMASK0_TXBUEM                                                      1 /* Transmit Buffer Underflow Error Mask. */
#define BITM_MAC_IMASK0_TXBUEM                                                      0x00000004 /* Transmit Buffer Underflow Error Mask. */
#define BITP_MAC_IMASK0_RXBOEM                                                      3 /* Receive Buffer Overflow Error Mask. */
#define BITL_MAC_IMASK0_RXBOEM                                                      1 /* Receive Buffer Overflow Error Mask. */
#define BITM_MAC_IMASK0_RXBOEM                                                      0x00000008 /* Receive Buffer Overflow Error Mask. */
#define BITP_MAC_IMASK0_LOFEM                                                       4 /* Loss of Frame Error Mask. */
#define BITL_MAC_IMASK0_LOFEM                                                       1 /* Loss of Frame Error Mask. */
#define BITM_MAC_IMASK0_LOFEM                                                       0x00000010 /* Loss of Frame Error Mask. */
#define BITP_MAC_IMASK0_HDREM                                                       5 /* Header Error Mask. */
#define BITL_MAC_IMASK0_HDREM                                                       1 /* Header Error Mask. */
#define BITM_MAC_IMASK0_HDREM                                                       0x00000020 /* Header Error Mask. */
#define BITP_MAC_IMASK0_RESETCM                                                     6 /* RESET Complete Mask. */
#define BITL_MAC_IMASK0_RESETCM                                                     1 /* RESET Complete Mask. */
#define BITM_MAC_IMASK0_RESETCM                                                     0x00000040/* RESET Complete Mask. */
#define BITP_MAC_IMASK0_PHYINTM                                                     7 /* Physical Layer Interrupt Mask. */
#define BITL_MAC_IMASK0_PHYINTM                                                     1 /* Physical Layer Interrupt Mask. */
#define BITM_MAC_IMASK0_PHYINTM                                                     0x00000080 /* Physical Layer Interrupt Mask. */
#define BITP_MAC_IMASK0_TTSCAAM                                                     8 /* Transmit Timestamp Capture Available A Mask. */
#define BITL_MAC_IMASK0_TTSCAAM                                                     1 /* Transmit Timestamp Capture Available A Mask. */
#define BITM_MAC_IMASK0_TTSCAAM                                                     0x00000100 /* Transmit Timestamp Capture Available A Mask. */
#define BITP_MAC_IMASK0_TTSCABM                                                     9 /* Transmit Timestamp Capture Available B Mask. */
#define BITL_MAC_IMASK0_TTSCABM                                                     1 /* Transmit Timestamp Capture Available B Mask. */
#define BITM_MAC_IMASK0_TTSCABM                                                     0x00000200 /* Transmit Timestamp Capture Available B Mask. */
#define BITP_MAC_IMASK0_TTSCACM                                                     10 /* Transmit Timestamp Capture Available C Mask. */
#define BITL_MAC_IMASK0_TTSCACM                                                     1 /* Transmit Timestamp Capture Available C Mask. */
#define BITM_MAC_IMASK0_TTSCACM                                                     0x00000400 /* Transmit Timestamp Capture Available C Mask. */
#define BITP_MAC_IMASK0_TXFCSEM                                                     11 /* Transmit Frame Check Sequence Error Mask. */
#define BITL_MAC_IMASK0_TXFCSEM                                                     1 /* Transmit Frame Check Sequence Error Mask. */
#define BITM_MAC_IMASK0_TXFCSEM                                                     0x00000800 /* Transmit Frame Check Sequence Error Mask. */
#define BITP_MAC_IMASK0_CDPEM                                                       12 /* Control Data Protection Error Mask. */
#define BITL_MAC_IMASK0_CDPEM                                                       1 /* Control Data Protection Error Mask. */
#define BITM_MAC_IMASK0_CDPEM                                                       0x00001000 /* Control Data Protection Error Mask. */

/* ----------------------------------------------------------------------------------------------------
          IMASK1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_IMASK1_LINK_CHANGE_MASK                                            1 /* Mask Bit for LINK_CHANGE. */
#define BITL_MAC_IMASK1_LINK_CHANGE_MASK                                            1 / * Mask Bit for LINK_CHANGE. */
#define BITM_MAC_IMASK1_LINK_CHANGE_MASK                                            0x00000002 /* Mask Bit for LINK_CHANGE. */
#define BITP_MAC_IMASK1_TX_RDY_MASK                                                 3 /* Mask Bit for TX_FRM_DONE. */
#define BITL_MAC_IMASK1_TX_RDY_MASK                                                 1 /* Mask Bit for TX_FRM_DONE. */
#define BITM_MAC_IMASK1_TX_RDY_MASK                                                 0x00000008 /* Mask Bit for TX_FRM_DONE. */
#define BITP_MAC_IMASK1_P1_RX_RDY_MASK                                              4 /* Mask Bit for P1_RX_RDY. */
#define BITL_MAC_IMASK1_P1_RX_RDY_MASK                                              1 /* Mask Bit for P1_RX_RDY. */
#define BITM_MAC_IMASK1_P1_RX_RDY_MASK                                              0x00000010 /* Mask Bit for P1_RX_RDY. */
#define BITP_MAC_IMASK1_P1_RX_IFG_ERR_MASK                                          8 /* Mask Bit for RX_IFG_ERR. */
#define BITL_MAC_IMASK1_P1_RX_IFG_ERR_MASK                                          1 /* Mask Bit for RX_IFG_ERR. */
#define BITM_MAC_IMASK1_P1_RX_IFG_ERR_MASK                                          0x00000100 /* Mask Bit for RX_IFG_ERR. */
#define BITP_MAC_IMASK1_SPI_ERR_MASK                                                10 /* Mask Bit for SPI_ERR. */
#define BITL_MAC_IMASK1_SPI_ERR_MASK                                                1 /* Mask Bit for SPI_ERR. */
#define BITM_MAC_IMASK1_SPI_ERR_MASK                                                0x00000400 /* Mask Bit for SPI_ERR. */
#define BITP_MAC_IMASK1_RX_ECC_ERR_MASK                                             11 /* Mask Bit for RXF_ECC_ERR. */
#define BITL_MAC_IMASK1_RX_ECC_ERR_MASK                                             1 /* Mask Bit for RXF_ECC_ERR. */
#define BITM_MAC_IMASK1_RX_ECC_ERR_MASK                                             0x00000800 /* Mask Bit for RXF_ECC_ERR. */
#define BITP_MAC_IMASK1_TX_ECC_ERR_MASK                                             12 /* Mask Bit for TXF_ECC_ERR. */
#define BITL_MAC_IMASK1_TX_ECC_ERR_MASK                                             1 /* Mask Bit for TXF_ECC_ERR. */
#define BITM_MAC_IMASK1_TX_ECC_ERR_MASK                                             0x00001000 /* Mask Bit for TXF_ECC_ERR. */
#define BITP_MAC_IMASK1_P2_RX_RDY_MASK                                              17
#define BITL_MAC_IMASK1_P2_RX_RDY_MASK                                              1
#define BITM_MAC_IMASK1_P2_RX_RDY_MASK                                              0x00020000
#define BITP_MAC_IMASK1_P2_PHYINT_MASK                                              19
#define BITL_MAC_IMASK1_P2_PHYINT_MASK                                              1
#define BITM_MAC_IMASK1_P2_PHYINT_MASK                                              0x00080000
#define BITP_MAC_IMASK1_P2_TTSCAAM                                                  20
#define BITL_MAC_IMASK1_P2_TTSCAAM                                                  1
#define BITM_MAC_IMASK1_P2_TTSCAAM                                                  0x00100000
#define BITP_MAC_IMASK1_P2_TTSCABM                                                  21
#define BITL_MAC_IMASK1_P2_TTSCABM                                                  1
#define BITM_MAC_IMASK1_P2_TTSCABM                                                  0x00200000
#define BITP_MAC_IMASK1_P2_TTSCACM                                                  22
#define BITL_MAC_IMASK1_P2_TTSCACM                                                  1
#define BITM_MAC_IMASK1_P2_TTSCACM                                                  0x00400000
#define BITP_MAC_IMASK1_P2_RX_IFG_ERR_MASK                                          23
#define BITL_MAC_IMASK1_P2_RX_IFG_ERR_MASK                                          1
#define BITM_MAC_IMASK1_P2_RX_IFG_ERR_MASK                                          0x00800000
#define BITP_MAC_IMASK1_P2_TXFCSEM                                                  24
#define BITL_MAC_IMASK1_P2_TXFCSEM                                                  1
#define BITM_MAC_IMASK1_P2_TXFCSEM                                                  0x01000000

/* ----------------------------------------------------------------------------------------------------
          TTSCAH
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TTSCAH_TTSCH_A                                                     0 /* Transmit Timestamp A Bits 63-32. */
#define BITL_MAC_TTSCAH_TTSCH_A                                                     32 /* Transmit Timestamp A Bits 63-32. */
#define BITM_MAC_TTSCAH_TTSCH_A                                                     0xFFFFFFFF /* Transmit Timestamp A Bits 63-32. */

/* ----------------------------------------------------------------------------------------------------
          TTSCAL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TTSCAL_TTSCL_A                                                     0 /* Transmit Timestamp A Bits 31-0. */
#define BITL_MAC_TTSCAL_TTSCL_A                                                     32 /* Transmit Timestamp A Bits 31-0. */
#define BITM_MAC_TTSCAL_TTSCL_A                                                     0xFFFFFFFF /* Transmit Timestamp A Bits 31-0. */

/* ----------------------------------------------------------------------------------------------------
          TTSCBH
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TTSCBH_TTSCH_B                                                     0 /* Transmit Timestamp B Bits 63-32. */
#define BITL_MAC_TTSCBH_TTSCH_B                                                     32 /* Transmit Timestamp B Bits 63-32. */
#define BITM_MAC_TTSCBH_TTSCH_B                                                     0xFFFFFFFF /* Transmit Timestamp B Bits 63-32. */

/* ----------------------------------------------------------------------------------------------------
          TTSCBL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TTSCBL_TTSCL_B                                                     0 /* Transmit Timestamp B Bits 31-0. */
#define BITL_MAC_TTSCBL_TTSCL_B                                                     32 /* Transmit Timestamp B Bits 31-0. */
#define BITM_MAC_TTSCBL_TTSCL_B                                                     0xFFFFFFFF /* Transmit Timestamp B Bits 31-0. */

/* ----------------------------------------------------------------------------------------------------
          TTSCCH
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TTSCCH_TTSCH_C                                                     0 /* Transmit Timestamp C Bits 63-32. */
#define BITL_MAC_TTSCCH_TTSCH_C                                                     32 /* Transmit Timestamp C Bits 63-32. */
#define BITM_MAC_TTSCCH_TTSCH_C                                                     0xFFFFFFFF /* Transmit Timestamp C Bits 63-32. */

/* ----------------------------------------------------------------------------------------------------
          TTSCCL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TTSCCL_TTSCL_C                                                     0 /* Transmit Timestamp C Bits 31-0. */
#define BITL_MAC_TTSCCL_TTSCL_C                                                     32 /* Transmit Timestamp C Bits 31-0. */
#define BITM_MAC_TTSCCL_TTSCL_C                                                     0xFFFFFFFF /* Transmit Timestamp C Bits 31-0. */

/* ----------------------------------------------------------------------------------------------------
          MDIOACC_0_
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_MDIOACC_N__MDIO_DATA                                               0 /* Data/Address Value. */
#define BITL_MAC_MDIOACC_N__MDIO_DATA                                               16 /* Data/Address Value. */
#define BITM_MAC_MDIOACC_N__MDIO_DATA                                               0x0000FFFF /* Data/Address Value. */
#define BITP_MAC_MDIOACC_N__MDIO_DEVAD                                              16 /* MDIO Device Address. */
#define BITL_MAC_MDIOACC_N__MDIO_DEVAD                                              5 /* MDIO Device Address. */
#define BITM_MAC_MDIOACC_N__MDIO_DEVAD                                              0x001F0000 /* MDIO Device Address. */
#define BITP_MAC_MDIOACC_N__MDIO_PRTAD                                              21 /* MDIO Port Address. */
#define BITL_MAC_MDIOACC_N__MDIO_PRTAD                                              5 /* MDIO Port Address. */
#define BITM_MAC_MDIOACC_N__MDIO_PRTAD                                              0x03E00000 /* MDIO Port Address. */
#define BITP_MAC_MDIOACC_N__MDIO_OP                                                 26 /* Operation Code. */
#define BITL_MAC_MDIOACC_N__MDIO_OP                                                 2 /* Operation Code. */
#define BITM_MAC_MDIOACC_N__MDIO_OP                                                 0x0C000000 /* Operation Code. */
#define BITP_MAC_MDIOACC_N__MDIO_ST                                                 28 /* Start of Frame. */
#define BITL_MAC_MDIOACC_N__MDIO_ST                                                 2 /* Start of Frame. */
#define BITM_MAC_MDIOACC_N__MDIO_ST                                                 0x30000000 /* Start of Frame. */
#define BITP_MAC_MDIOACC_N__MDIO_TAERR                                              30 /* Turnaround Error. */
#define BITL_MAC_MDIOACC_N__MDIO_TAERR                                              1 /* Turnaround Error. */
#define BITM_MAC_MDIOACC_N__MDIO_TAERR                                              0x40000000 /* Turnaround Error. */
#define BITP_MAC_MDIOACC_N__MDIO_TRDONE                                             31 /* Transaction Done. */
#define BITL_MAC_MDIOACC_N__MDIO_TRDONE                                             1 /* Transaction Done. */
#define BITM_MAC_MDIOACC_N__MDIO_TRDONE                                             0x80000000 /* Transaction Done. */

#define ENUM_MAC_MDIOACC_N__MDIO_ST_CLAUSE22                                        0x00000001 /* No description provided */
#define ENUM_MAC_MDIOACC_N__MDIO_ST_CLAUSE45                                        0x00000000 /* No description provided */
#define ENUM_MAC_MDIOACC_N__MDIO_OP_MD_ADDR                                         0x00000000 /* MD Address Command. */
#define ENUM_MAC_MDIOACC_N__MDIO_OP_MD_WR                                           0x00000001 /* Write Command. */
#define ENUM_MAC_MDIOACC_N__MDIO_OP_MD_RD                                           0x00000003 /* Read Command. */
#define ENUM_MAC_MDIOACC_N__MDIO_OP_MD_INC_RD                                       0x00000002 /* Incremental Read Command. */

/* ----------------------------------------------------------------------------------------------------
          TX_FSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TX_FSIZE_TX_FRM_SIZE                                               0 /* Transmit Frame Size. */
#define BITL_MAC_TX_FSIZE_TX_FRM_SIZE                                               11 /* Transmit Frame Size. */
#define BITM_MAC_TX_FSIZE_TX_FRM_SIZE                                               0x000007FF /* Transmit Frame Size. */

/* ----------------------------------------------------------------------------------------------------
          TX
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TX_TDR                                                             0 /* Transmit Data Register. */
#define BITL_MAC_TX_TDR                                                             32 /* Transmit Data Register. */
#define BITM_MAC_TX_TDR                                                             0xFFFFFFFF /* Transmit Data Register. */

/* ----------------------------------------------------------------------------------------------------
          TX_SPACE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TX_SPACE_TX_SPACE                                                  0 /* Transmit FIFO Space Available in Half Words (16 Bits). */
#define BITL_MAC_TX_SPACE_TX_SPACE                                                  14 /* Transmit FIFO Space Available in Half Words (16 Bits). */
#define BITM_MAC_TX_SPACE_TX_SPACE                                                  0x00003FFF /* Transmit FIFO Space Available in Half Words (16 Bits). */

/* ----------------------------------------------------------------------------------------------------
          TX_THRESH
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TX_THRESH_HOST_TX_THRESH                                           0 /* Host Transmit Start Threshold in Cut Through. */
#define BITL_MAC_TX_THRESH_HOST_TX_THRESH                                           6 /* Host Transmit Start Threshold in Cut Through. */
#define BITM_MAC_TX_THRESH_HOST_TX_THRESH                                           0x0000003F /* Host Transmit Start Threshold in Cut Through. */
#define BITP_MAC_TX_THRESH_P2P_TX_THRESH                                            6
#define BITL_MAC_TX_THRESH_P2P_TX_THRESH                                            6
#define BITM_MAC_TX_THRESH_P2P_TX_THRESH                                            0x00000FC0

/* ----------------------------------------------------------------------------------------------------
          TX_PRI
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TX_PRI_TX_HOST_PRI                                                 0
#define BITL_MAC_TX_PRI_TX_HOST_PRI                                                 3
#define BITM_MAC_TX_PRI_TX_HOST_PRI                                                 0x00000007

#define ENUM_MAC_TX_PRI_50_50_HOST_PORT                                             0x00000000
#define ENUM_MAC_TX_PRI_100_0_HOST_PORT                                             0x00000005 /* No description provided */
#define ENUM_MAC_TX_PRI_0_100_HOST_PORT                                             0x00000006 /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FIFO_CLR
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_FIFO_CLR_MAC_RXF_CLR                                               0 /* Clear the Receive FIFO(s). */
#define BITL_MAC_FIFO_CLR_MAC_RXF_CLR                                               1 /* Clear the Receive FIFO(s). */
#define BITM_MAC_FIFO_CLR_MAC_RXF_CLR                                               0x00000001 /* Clear the Receive FIFO(s). */
#define BITP_MAC_FIFO_CLR_MAC_LES_FIFOS_CLR                                         2
#define BITL_MAC_FIFO_CLR_MAC_LES_FIFOS_CLR                                         1
#define BITM_MAC_FIFO_CLR_MAC_LES_FIFOS_CLR                                         0x00000004
#define BITP_MAC_FIFO_CLR_MAC_LES_P2P_FIFOS_CLR                                     3
#define BITL_MAC_FIFO_CLR_MAC_LES_P2P_FIFOS_CLR                                     1
#define BITM_MAC_FIFO_CLR_MAC_LES_P2P_FIFOS_CLR                                     0x00000008

/* ----------------------------------------------------------------------------------------------------
          SCRATCH_N_
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_SCRATCH_N__SCRATCH_DATA                                            0 /* Scratch Data. */
#define BITL_MAC_SCRATCH_N__SCRATCH_DATA                                            32 /* Scratch Data. */
#define BITM_MAC_SCRATCH_N__SCRATCH_DATA                                            0xFFFFFFFF /* Scratch Data. */

/* ----------------------------------------------------------------------------------------------------
          MAC_RST_STATUS
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_MAC_RST_STATUS_MAC_OSC_CLK_RDY                                     0 /* MAC Oscillator Clock Ready. */
#define BITL_MAC_MAC_RST_STATUS_MAC_OSC_CLK_RDY                                     1 /* MAC Oscillator Clock Ready. */
#define BITM_MAC_MAC_RST_STATUS_MAC_OSC_CLK_RDY                                     0x00000001 /* MAC Oscillator Clock Ready. */
#define BITP_MAC_MAC_RST_STATUS_MAC_CRYSL_CLK_RDY                                   1 /* MAC Crystal Clock Ready. */
#define BITL_MAC_MAC_RST_STATUS_MAC_CRYSL_CLK_RDY                                   1 /* MAC Crystal Clock Ready. */
#define BITM_MAC_MAC_RST_STATUS_MAC_CRYSL_CLK_RDY                                   0x00000002 /* MAC Crystal Clock Ready. */

/* ----------------------------------------------------------------------------------------------------
          SOFT_RST
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_SOFT_RST_RST_KEY                                                   0 /* Software Reset. */
#define BITL_MAC_SOFT_RST_RST_KEY                                                   16 /* Software Reset. */
#define BITM_MAC_SOFT_RST_RST_KEY                                                   0x0000FFFF /* Software Reset. */

#define ENUM_MAC_SOFT_RST_RST_KEY_RST_MAC_ONLY_KEY1                                 0x00004F1C /* Key1 to Reset the MAC Logic Only. */
#define ENUM_MAC_SOFT_RST_RST_KEY_RST_MAC_ONLY_KEY2                                 0x0000C1F4 /* Key 2 to Reset the MAC Logic Only. */
#define ENUM_MAC_SOFT_RST_RST_KEY_MAC_RST_EXIT_REQ_KEY1                             0x00006F1A /* Key 1 to Request Release of Reset to the MAC Core Logic. */
#define ENUM_MAC_SOFT_RST_RST_KEY_MAC_RST_EXIT_REQ_KEY2                             0x0000A1F6 /* Key 2 to Request Release of Reset to the MAC Core Logic. */

/* ----------------------------------------------------------------------------------------------------
          SPI_INJ_ERR
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_SPI_INJ_ERR_TEST_SPI_INJ_ERR                                       0 /* Inject an Error on the SPI MISO Path. */
#define BITL_MAC_SPI_INJ_ERR_TEST_SPI_INJ_ERR                                       1 /* Inject an Error on the SPI MISO Path. */
#define BITM_MAC_SPI_INJ_ERR_TEST_SPI_INJ_ERR                                       0x00000001 /* Inject an Error on the SPI MISO Path. */

/* ----------------------------------------------------------------------------------------------------
          FIFO_SIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_FIFO_SIZE_HTX_SIZE                                                 0 /* Host Transmit FIFO Size. */
#define BITL_MAC_FIFO_SIZE_HTX_SIZE                                                 4 /* Host Transmit FIFO Size. */
#define BITM_MAC_FIFO_SIZE_HTX_SIZE                                                 0x0000000F /* Host Transmit FIFO Size. */
#define BITP_MAC_FIFO_SIZE_P1_RX_LO_SIZE                                            4 /* Port 1 Rx Low Priority FIFO Size. */
#define BITL_MAC_FIFO_SIZE_P1_RX_LO_SIZE                                            4 /* Port 1 Rx Low Priority FIFO Size. */
#define BITM_MAC_FIFO_SIZE_P1_RX_LO_SIZE                                            0x000000F0 /* Port 1 Rx Low Priority FIFO Size. */
#define BITP_MAC_FIFO_SIZE_P1_RX_HI_SIZE                                            8 /* Port 1 Rx High Priority FIFO Size. */
#define BITL_MAC_FIFO_SIZE_P1_RX_HI_SIZE                                            4 /* Port 1 Rx High Priority FIFO Size. */
#define BITM_MAC_FIFO_SIZE_P1_RX_HI_SIZE                                            0x00000F00 /* Port 1 Rx High Priority FIFO Size. */
#define BITP_MAC_FIFO_SIZE_P2_RX_LO_SIZE                                            12
#define BITL_MAC_FIFO_SIZE_P2_RX_LO_SIZE                                            4
#define BITM_MAC_FIFO_SIZE_P2_RX_LO_SIZE                                            0x0000F000
#define BITP_MAC_FIFO_SIZE_P2_RX_HI_SIZE                                            16
#define BITL_MAC_FIFO_SIZE_P2_RX_HI_SIZE                                            4
#define BITM_MAC_FIFO_SIZE_P2_RX_HI_SIZE                                            0x000F0000
#define BITP_MAC_FIFO_SIZE_P1_TX_SIZE                                               20
#define BITL_MAC_FIFO_SIZE_P1_TX_SIZE                                               4
#define BITM_MAC_FIFO_SIZE_P1_TX_SIZE                                               0x00F00000
#define BITP_MAC_FIFO_SIZE_P2_RX_SIZE                                               24
#define BITL_MAC_FIFO_SIZE_P2_RX_SIZE                                               4
#define BITM_MAC_FIFO_SIZE_P2_RX_SIZE                                               0x0F000000

#define ENUM_MAC_FIFO_SIZE_P2_TX_SIZE_TXSIZE_0K                                     0x00000000 /* 0K Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_TX_SIZE_TXSIZE_2K                                     0x00000001 /* 2K Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_TX_SIZE_TXSIZE_4K                                     0x00000002 /* 4k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_TX_SIZE_TXSIZE_6K                                     0x00000003 /* 6k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_TX_SIZE_TXSIZE_8K                                     0x00000004 /* 8k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_TX_SIZE_TXSIZE_10K                                    0x00000005 /* 10k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_TX_SIZE_TXSIZE_12K                                    0x00000006 /* 12k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_TX_SIZE_TXSIZE_14K                                    0x00000007 /* 14k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_TX_SIZE_TXSIZE_16K                                    0x00000008 /* 16k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_TX_SIZE_TXSIZE_0K                                     0x00000000 /* 0K Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_TX_SIZE_TXSIZE_2K                                     0x00000001 /* 2k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_TX_SIZE_TXSIZE_4K                                     0x00000002 /* 4k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_TX_SIZE_TXSIZE_6K                                     0x00000003 /* 6k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_TX_SIZE_TXSIZE_8K                                     0x00000004 /* 8k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_TX_SIZE_TXSIZE_10K                                    0x00000005 /* 10k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_TX_SIZE_TXSIZE_12K                                    0x00000006 /* 12k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_TX_SIZE_TXSIZE_14K                                    0x00000007 /* 14k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_TX_SIZE_TXSIZE_16K                                    0x00000008 /* 16k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_HI_SIZE_RXSIZE_0K                                  0x00000000 /* 0K Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_HI_SIZE_RXSIZE_2K                                  0x00000001 /* 2K Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_HI_SIZE_RXSIZE_4K                                  0x00000002 /* 4k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_HI_SIZE_RXSIZE_6K                                  0x00000003 /* 6k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_HI_SIZE_RXSIZE_8K                                  0x00000004 /* 8k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_HI_SIZE_RXSIZE_10K                                 0x00000005 /* 10k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_HI_SIZE_RXSIZE_12K                                 0x00000006 /* 12k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_HI_SIZE_RXSIZE_14K                                 0x00000007 /* 14k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_HI_SIZE_RXSIZE_16K                                 0x00000008 /* 16k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_LO_SIZE_RXSIZE_0K                                  0x00000000 /* 0K Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_LO_SIZE_RXSIZE_2K                                  0x00000001 /* 2k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_LO_SIZE_RXSIZE_4K                                  0x00000002 /* 4k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_LO_SIZE_RXSIZE_6K                                  0x00000003 /* 6k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_LO_SIZE_RXSIZE_8K                                  0x00000004 /* 8k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_LO_SIZE_RXSIZE_10K                                 0x00000005 /* 10k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_LO_SIZE_RXSIZE_12K                                 0x00000006 /* 12k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_LO_SIZE_RXSIZE_14K                                 0x00000007 /* 14k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P2_RX_LO_SIZE_RXSIZE_16K                                 0x00000008 /* 16k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_0K                                  0x00000000 /* 0K Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_2K                                  0x00000001 /* 2K Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_4K                                  0x00000002 /* 4k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_6K                                  0x00000003 /* 6k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_8K                                  0x00000004 /* 8k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_10K                                 0x00000005 /* 10k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_12K                                 0x00000006 /* 12k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_14K                                 0x00000007 /* 14k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_16K                                 0x00000008 /* 16k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_0K                                  0x00000000 /* 0K Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_2K                                  0x00000001 /* 2k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_4K                                  0x00000002 /* 4k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_6K                                  0x00000003 /* 6k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_8K                                  0x00000004 /* 8k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_10K                                 0x00000005 /* 10k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_12K                                 0x00000006 /* 12k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_14K                                 0x00000007 /* 14k Bytes. */
#define ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_16K                                 0x00000008 /* 16k Bytes. */
#define ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_0K                                       0x00000000 /* 0k Bytes. */
#define ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_2K                                       0x00000001 /* 2k Bytes. */
#define ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_4K                                       0x00000002 /* 4k Bytes. */
#define ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_6K                                       0x00000003 /* 6k Bytes. */
#define ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_8K                                       0x00000004 /* 8k Bytes. */
#define ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_10K                                      0x00000005 /* 10k Bytes. */
#define ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_12K                                      0x00000006 /* 12k Bytes. */
#define ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_14K                                      0x00000007 /* 14k Bytes. */
#define ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_16K                                      0x00000008 /* 16k Byte. */

/* ----------------------------------------------------------------------------------------------------
          TFC
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TFC_TFC                                                            0 /* Number of Frames in the Tx FIFO. */
#define BITL_MAC_TFC_TFC                                                            9 /* Number of Frames in the Tx FIFO. */
#define BITM_MAC_TFC_TFC                                                            0x000001FF /* Number of Frames in the Tx FIFO. */

/* ----------------------------------------------------------------------------------------------------
          TXSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TXSIZE_TX_SIZE                                                     0 /* Data in the Tx FIFO. Number of Half Words(16 Bit). */
#define BITL_MAC_TXSIZE_TX_SIZE                                                     14 /* Data in the Tx FIFO. Number of Half Words(16 Bit). */
#define BITM_MAC_TXSIZE_TX_SIZE                                                     0x00003FFF /* Data in the Tx FIFO. Number of Half Words(16 Bit). */

/* ----------------------------------------------------------------------------------------------------
          HTX_OVF_FRM_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_HTX_OVF_FRM_CNT_HTX_OVF_FRM_CNT                                    0 /* Counts Host Tx Frames Dropped Due to FIFO Overflow. */
#define BITL_MAC_HTX_OVF_FRM_CNT_HTX_OVF_FRM_CNT                                    32 /* Counts Host Tx Frames Dropped Due to FIFO Overflow. */
#define BITM_MAC_HTX_OVF_FRM_CNT_HTX_OVF_FRM_CNT                                    0xFFFFFFFF /* Counts Host Tx Frames Dropped Due to FIFO Overflow. */

/* ----------------------------------------------------------------------------------------------------
          MECC_ERR_ADDR 
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_MECC_ERR_ADDR_MECC_ERR_ADDR                                        0 /* Address of an Uncorrectable ECC Error in Memory. */
#define BITL_MAC_MECC_ERR_ADDR_MECC_ERR_ADDR                                        14 /* Address of an Uncorrectable ECC Error in Memory. */
#define BITM_MAC_MECC_ERR_ADDR_MECC_ERR_ADDR                                        0x00003FFF /* Address of an Uncorrectable ECC Error in Memory. */

/* ----------------------------------------------------------------------------------------------------
          CECC_ERR_0_
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_CECC_ERR_N__CECC_ERR_CNT                                           0 /* Corrected ECC Error Count. */
#define BITL_MAC_CECC_ERR_N__CECC_ERR_CNT                                           10 /* Corrected ECC Error Count. */
#define BITM_MAC_CECC_ERR_N__CECC_ERR_CNT                                           0x000003FF /* Corrected ECC Error Count. */

/* ----------------------------------------------------------------------------------------------------
          ADDR_FILT_UPR_0_
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_ADDR_FILT_UPR_N__MAC_ADDR                                          0 /* MAC Address. */
#define BITL_MAC_ADDR_FILT_UPR_N__MAC_ADDR                                          16 /* MAC Address. */
#define BITM_MAC_ADDR_FILT_UPR_N__MAC_ADDR                                          0x0000FFFF /* MAC Address. */
#define BITP_MAC_ADDR_FILT_UPR_N__TO_HOST                                           16 /* Forward Frames Matching This MAC Address to the Host. */
#define BITL_MAC_ADDR_FILT_UPR_N__TO_HOST                                           1 /* Forward Frames Matching This MAC Address to the Host. */
#define BITM_MAC_ADDR_FILT_UPR_N__TO_HOST                                           0x00010000 /* Forward Frames Matching This MAC Address to the Host. */
#define BITP_MAC_ADDR_FILT_UPR_N__TO_OTHER_DEVICE                                   17
#define BITL_MAC_ADDR_FILT_UPR_N__TO_OTHER_DEVICE                                   1
#define BITM_MAC_ADDR_FILT_UPR_N__TO_OTHER_DEVICE                                   0x00020000
#define BITP_MAC_ADDR_FILT_UPR_N__HOST_PRI                                          19 /* Host Rx Port Priority. */
#define BITL_MAC_ADDR_FILT_UPR_N__HOST_PRI                                          1 /* Host Rx Port Priority. */
#define BITM_MAC_ADDR_FILT_UPR_N__HOST_PRI                                          0x00080000 /* Host Rx Port Priority. */
#define BITP_MAC_ADDR_FILT_UPR_N__APPLY2PORT1                                       30 /* Apply to Port 1. */
#define BITL_MAC_ADDR_FILT_UPR_N__APPLY2PORT1                                       1 /* Apply to Port 1. */
#define BITM_MAC_ADDR_FILT_UPR_N__APPLY2PORT1                                       0x40000000 /* Apply to Port 1. */
#define BITP_MAC_ADDR_FILT_UPR_N__APPLY2PORT2                                       31 /* Apply to Port 2. */
#define BITL_MAC_ADDR_FILT_UPR_N__APPLY2PORT2                                       1 /* Apply to Port 2. */
#define BITM_MAC_ADDR_FILT_UPR_N__APPLY2PORT2                                       0x80000000 /* Apply to Port 2. */

#define ENUM_MAC_ADDR_FILT_UPR_N__APPLY2PORT2_NOTAPPLY                              0x00000000 /* Do Not Apply to Port 1. */
#define ENUM_MAC_ADDR_FILT_UPR_N__APPLY2PORT2_APPLY                                 0x00000001 /* Apply to Port 1. */
#define ENUM_MAC_ADDR_FILT_UPR_N__APPLY2PORT1_NOTAPPLY                              0x00000000 /* Do Not Apply to Port 1. */
#define ENUM_MAC_ADDR_FILT_UPR_N__APPLY2PORT1_APPLY                                 0x00000001 /* Apply to Port 1. */

/* ----------------------------------------------------------------------------------------------------
          ADDR_FILT_LWR_0_
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_ADDR_FILT_LWR_N__MAC_ADDR                                          0 /* MAC Address. */
#define BITL_MAC_ADDR_FILT_LWR_N__MAC_ADDR                                          32 /* MAC Address. */
#define BITM_MAC_ADDR_FILT_LWR_N__MAC_ADDR                                          0xFFFFFFFF /* MAC Address. */

/* ----------------------------------------------------------------------------------------------------
          ADDR_MSK_UPR_0_
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_ADDR_MSK_UPR_N__MAC_ADDR_MASK                                      0 /* MAC Address Bit Mask for the Address Table. */
#define BITL_MAC_ADDR_MSK_UPR_N__MAC_ADDR_MASK                                      16 /* MAC Address Bit Mask for the Address Table. */
#define BITM_MAC_ADDR_MSK_UPR_N__MAC_ADDR_MASK                                      0x0000FFFF /* MAC Address Bit Mask for the Address Table. */

/* ----------------------------------------------------------------------------------------------------
          ADDR_MSK_LWR_0_
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_ADDR_MSK_LWR_N__MAC_ADDR_MASK                                      0 /* MAC Address Bit Mask for the Address Table. */
#define BITL_MAC_ADDR_MSK_LWR_N__MAC_ADDR_MASK                                      32 /* MAC Address Bit Mask for the Address Table. */
#define BITM_MAC_ADDR_MSK_LWR_N__MAC_ADDR_MASK                                      0xFFFFFFFF /* MAC Address Bit Mask for the Address Table. */

/* ----------------------------------------------------------------------------------------------------
          TS_ADDEND
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_ADDEND_TS_ADDEND                                                0 /* Timestamp Accumulator Addend. */
#define BITL_MAC_TS_ADDEND_TS_ADDEND                                                32 /* Timestamp Accumulator Addend. */
#define BITM_MAC_TS_ADDEND_TS_ADDEND                                                0xFFFFFFFF /* Timestamp Accumulator Addend. */

/* ----------------------------------------------------------------------------------------------------
          TS_1SEC_CMP
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_1SEC_CMP_TS_1SEC_CMP                                            0 /* Timestamp 1 Second Compare Value. */
#define BITL_MAC_TS_1SEC_CMP_TS_1SEC_CMP                                            32 /* Timestamp 1 Second Compare Value. */
#define BITM_MAC_TS_1SEC_CMP_TS_1SEC_CMP                                            0xFFFFFFFF /* Timestamp 1 Second Compare Value. */

/* ----------------------------------------------------------------------------------------------------
          TS_SEC_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_SEC_CNT_TS_SEC_CNT                                              0 /* Write to the Seconds Counter. */
#define BITL_MAC_TS_SEC_CNT_TS_SEC_CNT                                              32 /* Write to the Seconds Counter. */
#define BITM_MAC_TS_SEC_CNT_TS_SEC_CNT                                              0xFFFFFFFF /* Write to the Seconds Counter. */

/* ----------------------------------------------------------------------------------------------------
          TS_NS_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_NS_CNT_TS_NS_CNT                                                0 /* Write to the Nanoseconds Counter. */
#define BITL_MAC_TS_NS_CNT_TS_NS_CNT                                                32 /* Write to the Nanoseconds Counter. */
#define BITM_MAC_TS_NS_CNT_TS_NS_CNT                                                0xFFFFFFFF /* Write to the Nanoseconds Counter. */

/* ----------------------------------------------------------------------------------------------------
          TS_CFG
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_CFG_TS_EN                                                       0 /* Enable the 1588 Timestamp Counter. */
#define BITL_MAC_TS_CFG_TS_EN                                                       1 /* Enable the 1588 Timestamp Counter. */
#define BITM_MAC_TS_CFG_TS_EN                                                       0x00000001 /* Enable the 1588 Timestamp Counter. */
#define BITP_MAC_TS_CFG_TS_CLR                                                      1 /* Clear the 1588 Timestamp Counters. */
#define BITL_MAC_TS_CFG_TS_CLR                                                      1 /* Clear the 1588 Timestamp Counters. */
#define BITM_MAC_TS_CFG_TS_CLR                                                      0x00000002 /* Clear the 1588 Timestamp Counters. */
#define BITP_MAC_TS_CFG_TS_TIMER_DEF                                                2 /* The Default Value for the TS_TIMER Output. */
#define BITL_MAC_TS_CFG_TS_TIMER_DEF                                                1 /* The Default Value for the TS_TIMER Output. */
#define BITM_MAC_TS_CFG_TS_TIMER_DEF                                                0x00000004 /* The Default Value for the TS_TIMER Output. */
#define BITP_MAC_TS_CFG_TS_TIMER_STOP                                               3 /* Stop Toggling the TS_TIMER Output. */
#define BITL_MAC_TS_CFG_TS_TIMER_STOP                                               1 /* Stop Toggling the TS_TIMER Output. */
#define BITM_MAC_TS_CFG_TS_TIMER_STOP                                               0x00000008 /* Stop Toggling the TS_TIMER Output. */
#define BITP_MAC_TS_CFG_TS_CAPT_FREE_CNT                                            4  /* Capture the Free Running Counter. */
#define BITL_MAC_TS_CFG_TS_CAPT_FREE_CNT                                            1 /* Capture the Free Running Counter. */
#define BITM_MAC_TS_CFG_TS_CAPT_FREE_CNT                                            0x00000010 /* Capture the Free Running Counter. */

/* ----------------------------------------------------------------------------------------------------
          TS_TIMER_HI
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_TIMER_HI_TS_TIMER_HI                                            0 /* TS_TIMER High Period. */
#define BITL_MAC_TS_TIMER_HI_TS_TIMER_HI                                            32 /* TS_TIMER High Period. */
#define BITM_MAC_TS_TIMER_HI_TS_TIMER_HI                                            0xFFFFFFFF /* TS_TIMER High Period. */

/* ----------------------------------------------------------------------------------------------------
          TS_TIMER_LO
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_TIMER_LO_TS_TIMER_LO                                            0 /* TS_TIMER Low Period. */
#define BITL_MAC_TS_TIMER_LO_TS_TIMER_LO                                            32 /* TS_TIMER Low Period. */
#define BITM_MAC_TS_TIMER_LO_TS_TIMER_LO                                            0xFFFFFFFF /* TS_TIMER Low Period. */

/* ----------------------------------------------------------------------------------------------------
          TS_TIMER_QE_CORR
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_TIMER_QE_CORR_TS_TIMER_QE_CORR                                  0 /* TS_TIMER Quantization Error Correction Value. */
#define BITL_MAC_TS_TIMER_QE_CORR_TS_TIMER_QE_CORR                                  8 /* TS_TIMER Quantization Error Correction Value. */
#define BITM_MAC_TS_TIMER_QE_CORR_TS_TIMER_QE_CORR                                  0x000000FF /* TS_TIMER Quantization Error Correction Value. */

/* ----------------------------------------------------------------------------------------------------
          TS_TIMER_START
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_TIMER_START_TS_TSTART                                           0 /* Point in Time at Which to Start the TS_TIMER Counter. */
#define BITL_MAC_TS_TIMER_START_TS_TSTART                                           32 /* Point in Time at Which to Start the TS_TIMER Counter. */
#define BITM_MAC_TS_TIMER_START_TS_TSTART                                           0xFFFFFFFF /* Point in Time at Which to Start the TS_TIMER Counter. */

/* ----------------------------------------------------------------------------------------------------
          TS_EXT_CAPT0
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_EXT_CAPT0_TS_EXT_CAPTD                                          0 /* Timestamp Captured on the Assertion of TS_CAPT Pin. */
#define BITL_MAC_TS_EXT_CAPT0_TS_EXT_CAPTD                                          32 /* Timestamp Captured on the Assertion of TS_CAPT Pin. */
#define BITM_MAC_TS_EXT_CAPT0_TS_EXT_CAPTD                                          0xFFFFFFFF /* Timestamp Captured on the Assertion of TS_CAPT Pin. */

/* ----------------------------------------------------------------------------------------------------
          TS_EXT_CAPT1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_EXT_CAPT1_TS_EXT_CAPTD                                          0 /* Timestamp Captured on the Assertion of TS_CAPT Pin. */
#define BITL_MAC_TS_EXT_CAPT1_TS_EXT_CAPTD                                          32 /* Timestamp Captured on the Assertion of TS_CAPT Pin. */
#define BITM_MAC_TS_EXT_CAPT1_TS_EXT_CAPTD                                          0xFFFFFFFF /* Timestamp Captured on the Assertion of TS_CAPT Pin. */

/* ----------------------------------------------------------------------------------------------------
          TS_FREECNT_CAPT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_TS_FREECNT_CAPT_TS_CNT_CAPTD                                       0 /* Captured Free Running Counter. */
#define BITL_MAC_TS_FREECNT_CAPT_TS_CNT_CAPTD                                       32 /* Captured Free Running Counter. */
#define BITM_MAC_TS_FREECNT_CAPT_TS_CNT_CAPTD                                       0xFFFFFFFF /* Captured Free Running Counter. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_FSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_FSIZE_P1_RX_FRM_SIZE                                         0 /* Receive Frame Size. */
#define BITL_MAC_P1_RX_FSIZE_P1_RX_FRM_SIZE                                         11 /* Receive Frame Size. */
#define BITM_MAC_P1_RX_FSIZE_P1_RX_FRM_SIZE                                         0x000007FF /* Receive Frame Size. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_P1_RDR                                                       0 /* Receive Data Register. */
#define BITL_MAC_P1_RX_P1_RDR                                                       32 /* Receive Data Register. */
#define BITM_MAC_P1_RX_P1_RDR                                                       0xFFFFFFFF /* Receive Data Register. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_FRM_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_FRM_CNT_P1_RX_FRM_CNT                                        0 /* Rx Frame Count. */
#define BITL_MAC_P1_RX_FRM_CNT_P1_RX_FRM_CNT                                        32 /* Rx Frame Count. */
#define BITM_MAC_P1_RX_FRM_CNT_P1_RX_FRM_CNT                                        0xFFFFFFFF /* Rx Frame Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_BCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_BCAST_CNT_P1_RX_BCAST_CNT                                    0 /* Rx Broadcast Frame Count. */
#define BITL_MAC_P1_RX_BCAST_CNT_P1_RX_BCAST_CNT                                    32 /* Rx Broadcast Frame Count. */
#define BITM_MAC_P1_RX_BCAST_CNT_P1_RX_BCAST_CNT                                    0xFFFFFFFF /* Rx Broadcast Frame Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_MCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_MCAST_CNT_P1_RX_MCAST_CNT                                    0 /* Rx Multicast Frame Count. */
#define BITL_MAC_P1_RX_MCAST_CNT_P1_RX_MCAST_CNT                                    32 /* Rx Multicast Frame Count. */
#define BITM_MAC_P1_RX_MCAST_CNT_P1_RX_MCAST_CNT                                    0xFFFFFFFF /* Rx Multicast Frame Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_UCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_UCAST_CNT_P1_RX_UCAST_CNT                                    0 /* Rx Unicast Frame Count. */
#define BITL_MAC_P1_RX_UCAST_CNT_P1_RX_UCAST_CNT                                    32 /* Rx Unicast Frame Count. */
#define BITM_MAC_P1_RX_UCAST_CNT_P1_RX_UCAST_CNT                                    0xFFFFFFFF /* Rx Unicast Frame Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_CRC_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_CRC_ERR_CNT_P1_RX_CRC_ERR_CNT                                0 /* Rx CRC Errored Frame Count. */
#define BITL_MAC_P1_RX_CRC_ERR_CNT_P1_RX_CRC_ERR_CNT                                32 /* Rx CRC Errored Frame Count. */
#define BITM_MAC_P1_RX_CRC_ERR_CNT_P1_RX_CRC_ERR_CNT                                0xFFFFFFFF /* Rx CRC Errored Frame Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_ALGN_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_ALGN_ERR_CNT_P1_RX_ALGN_ERR_CNT                              0 /* Rx Align Error Count. */
#define BITL_MAC_P1_RX_ALGN_ERR_CNT_P1_RX_ALGN_ERR_CNT                              32 /* Rx Align Error Count. */
#define BITM_MAC_P1_RX_ALGN_ERR_CNT_P1_RX_ALGN_ERR_CNT                              0xFFFFFFFF /* Rx Align Error Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_LS_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_LS_ERR_CNT_P1_RX_LS_ERR_CNT                                  0 /* Rx Long/Short Frame Error Count. */
#define BITL_MAC_P1_RX_LS_ERR_CNT_P1_RX_LS_ERR_CNT                                  32 /* Rx Long/Short Frame Error Count. */
#define BITM_MAC_P1_RX_LS_ERR_CNT_P1_RX_LS_ERR_CNT                                  0xFFFFFFFF /* Rx Long/Short Frame Error Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_PHY_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_PHY_ERR_CNT_P1_RX_PHY_ERR_CNT                                0 /* Rx PHY Error Count. */
#define BITL_MAC_P1_RX_PHY_ERR_CNT_P1_RX_PHY_ERR_CNT                                32 /* Rx PHY Error Count. */
#define BITM_MAC_P1_RX_PHY_ERR_CNT_P1_RX_PHY_ERR_CNT                                0xFFFFFFFF /* Rx PHY Error Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_TX_FRM_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_TX_FRM_CNT_P1_TX_FRM_CNT                                        0 /* Tx Frame Count. */
#define BITL_MAC_P1_TX_FRM_CNT_P1_TX_FRM_CNT                                        32 /* Tx Frame Count. */
#define BITM_MAC_P1_TX_FRM_CNT_P1_TX_FRM_CNT                                        0xFFFFFFFF /* Tx Frame Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_TX_BCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_TX_BCAST_CNT_P1_TX_BCAST_CNT                                    0 /* Tx Broadcast Frame Count. */
#define BITL_MAC_P1_TX_BCAST_CNT_P1_TX_BCAST_CNT                                    32 /* Tx Broadcast Frame Count. */
#define BITM_MAC_P1_TX_BCAST_CNT_P1_TX_BCAST_CNT                                    0xFFFFFFFF /* Tx Broadcast Frame Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_TX_MCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_TX_MCAST_CNT_P1_TX_MCAST_CNT                                    0 /* Tx Multicast Frame Count. */
#define BITL_MAC_P1_TX_MCAST_CNT_P1_TX_MCAST_CNT                                    32 /* Tx Multicast Frame Count. */
#define BITM_MAC_P1_TX_MCAST_CNT_P1_TX_MCAST_CNT                                    0xFFFFFFFF /* Tx Multicast Frame Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_TX_UCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_TX_UCAST_CNT_P1_TX_UCAST_CNT                                    0 /* Tx Unicast Frame Count. */
#define BITL_MAC_P1_TX_UCAST_CNT_P1_TX_UCAST_CNT                                    32 /* Tx Unicast Frame Count. */
#define BITM_MAC_P1_TX_UCAST_CNT_P1_TX_UCAST_CNT                                    0xFFFFFFFF /* Tx Unicast Frame Count. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_DROP_FULL_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_DROP_FULL_CNT_P1_RX_DROP_FULL_CNT                            0 /* Rx Frames Dropped Due to FIFO Full. */
#define BITL_MAC_P1_RX_DROP_FULL_CNT_P1_RX_DROP_FULL_CNT                            32 /* Rx Frames Dropped Due to FIFO Full. */
#define BITM_MAC_P1_RX_DROP_FULL_CNT_P1_RX_DROP_FULL_CNT                            0xFFFFFFFF /* Rx Frames Dropped Due to FIFO Full. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_DROP_FILT_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_DROP_FILT_CNT_P1_RX_DROP_FILT_CNT                            0 /* Rx Frames Dropped Due to Filtering. */
#define BITL_MAC_P1_RX_DROP_FILT_CNT_P1_RX_DROP_FILT_CNT                            32 /* Rx Frames Dropped Due to Filtering. */
#define BITM_MAC_P1_RX_DROP_FILT_CNT_P1_RX_DROP_FILT_CNT                            0xFFFFFFFF /* Rx Frames Dropped Due to Filtering. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_IFG_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_IFG_ERR_CNT_P1_RX_IFG_ERR_CNT                                0 /* IFG Error Counter for Port 1 Received Frames. */
#define BITL_MAC_P1_RX_IFG_ERR_CNT_P1_RX_IFG_ERR_CNT                                32 /* IFG Error Counter for Port 1 Received Frames. */
#define BITM_MAC_P1_RX_IFG_ERR_CNT_P1_RX_IFG_ERR_CNT                                0xFFFFFFFF /* IFG Error Counter for Port 1 Received Frames. */

/* ----------------------------------------------------------------------------------------------------
          P1_TX_IFG
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_TX_IFG_P1_TX_IFG                                                0 /* Inter Frame Gap. */
#define BITL_MAC_P1_TX_IFG_P1_TX_IFG                                                8 /* Inter Frame Gap. */
#define BITM_MAC_P1_TX_IFG_P1_TX_IFG                                                0x000000FF /* Inter Frame Gap. */

/* ----------------------------------------------------------------------------------------------------
          P1_LOOP
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_LOOP_P1_LOOPBACK_EN                                             0 /* MAC Loopback. */
#define BITL_MAC_P1_LOOP_P1_LOOPBACK_EN                                             1 /* MAC Loopback. */
#define BITM_MAC_P1_LOOP_P1_LOOPBACK_EN                                             0x00000001 /* MAC Loopback. */

#define ENUM_MAC_P1_LOOP_P1_LOOPBACK_EN_LOOPB_DIS                                   0x00000000 /* Normal Operation - Loopback Disabled. */
#define ENUM_MAC_P1_LOOP_P1_LOOPBACK_EN_LOOPB_EN                                    0x00000001 /* Loopback Enabled. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_CRC_EN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_CRC_EN_P1_CRC_CHK_EN                                         0 /* CRC Check Enable on Receive. */
#define BITL_MAC_P1_RX_CRC_EN_P1_CRC_CHK_EN                                         1 /* CRC Check Enable on Receive. */
#define BITM_MAC_P1_RX_CRC_EN_P1_CRC_CHK_EN                                         0x00000001 /* CRC Check Enable on Receive. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_IFG
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_IFG_P1_RX_IFG                                                0 /* Inter Frame Gap. */
#define BITL_MAC_P1_RX_IFG_P1_RX_IFG                                                6 /* Inter Frame Gap. */
#define BITM_MAC_P1_RX_IFG_P1_RX_IFG                                                0x0000003F /* Inter Frame Gap. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_MAX_LEN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_MAX_LEN_P1_MAX_FRM_LEN                                       0 /* Max Frame Length on Receive. */
#define BITL_MAC_P1_RX_MAX_LEN_P1_MAX_FRM_LEN                                       16 /* Max Frame Length on Receive. */
#define BITM_MAC_P1_RX_MAX_LEN_P1_MAX_FRM_LEN                                       0x0000FFFF /* Max Frame Length on Receive. */

/* ----------------------------------------------------------------------------------------------------
          P1_RX_MIN_LEN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_RX_MIN_LEN_P1_MIN_FRM_LEN                                       0 /* Min Frame Length on Receive. */
#define BITL_MAC_P1_RX_MIN_LEN_P1_MIN_FRM_LEN                                       16 /* Min Frame Length on Receive. */
#define BITM_MAC_P1_RX_MIN_LEN_P1_MIN_FRM_LEN                                       0x0000FFFF /* Min Frame Length on Receive. */

/* ----------------------------------------------------------------------------------------------------
          P1_LO_RFC
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_LO_RFC_P1_LO_RFC                                                0 /* Receive Frame Count for the Low Priority FIFO. */
#define BITL_MAC_P1_LO_RFC_P1_LO_RFC                                                9 /* Receive Frame Count for the Low Priority FIFO. */
#define BITM_MAC_P1_LO_RFC_P1_LO_RFC                                                0x000001FF /* Receive Frame Count for the Low Priority FIFO. */

/* ----------------------------------------------------------------------------------------------------
          P1_HI_RFC
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_HI_RFC_P1_HI_RFC                                                0 /* Receive Frame Count for the High Priority FIFO. */
#define BITL_MAC_P1_HI_RFC_P1_HI_RFC                                                9 /* Receive Frame Count for the High Priority FIFO. */
#define BITM_MAC_P1_HI_RFC_P1_HI_RFC                                                0x000001FF /* Receive Frame Count for the High Priority FIFO. */

/* ----------------------------------------------------------------------------------------------------
          P1_LO_RXSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_LO_RXSIZE_P1_LO_RXSIZE                                          0 /* Data in the Rx FIFO. Number of Half Words(16 Bit). */
#define BITL_MAC_P1_LO_RXSIZE_P1_LO_RXSIZE                                          14 /* Data in the Rx FIFO. Number of Half Words(16 Bit). */
#define BITM_MAC_P1_LO_RXSIZE_P1_LO_RXSIZE                                          0x00003FFF /* Data in the Rx FIFO. Number of Half Words(16 Bit). */

/* ----------------------------------------------------------------------------------------------------
          P1_HI_RXSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1_HI_RXSIZE_P1_HI_RXSIZE                                          0 /* Data in the Rx FIFO. Number of Half Words(16 Bit). */
#define BITL_MAC_P1_HI_RXSIZE_P1_HI_RXSIZE                                          14 /* Data in the Rx FIFO. Number of Half Words(16 Bit). */
#define BITM_MAC_P1_HI_RXSIZE_P1_HI_RXSIZE                                          0x00003FFF /* Data in the Rx FIFO. Number of Half Words(16 Bit). */

/* ----------------------------------------------------------------------------------------------------
          P1TOP2_TXSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P1TOP2_TXSIZE_P1TOP2_TXSIZE                                        0
#define BITL_MAC_P1TOP2_TXSIZE_P1TOP2_TXSIZE                                        14
#define BITM_MAC_P1TOP2_TXSIZE_P1TOP2_TXSIZE                                        0x00003FFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_FSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_FSIZE_P2_RX_FRM_SIZE                                         0
#define BITL_MAC_P2_RX_FSIZE_P2_RX_FRM_SIZE                                         11
#define BITM_MAC_P2_RX_FSIZE_P2_RX_FRM_SIZE                                         0x000007FF

/* ----------------------------------------------------------------------------------------------------
          P2_RX
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_P2_RDR                                                       0
#define BITL_MAC_P2_RX_P2_RDR                                                       32
#define BITM_MAC_P2_RX_P2_RDR                                                       0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_FRM_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_FRM_CNT_P2_RX_FRM_CNT                                        0
#define BITL_MAC_P2_RX_FRM_CNT_P2_RX_FRM_CNT                                        32
#define BITM_MAC_P2_RX_FRM_CNT_P2_RX_FRM_CNT                                        0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_BCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_BCAST_CNT_P2_RX_BCAST_CNT                                    0
#define BITL_MAC_P2_RX_BCAST_CNT_P2_RX_BCAST_CNT                                    32
#define BITM_MAC_P2_RX_BCAST_CNT_P2_RX_BCAST_CNT                                    0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_MCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_MCAST_CNT_P2_RX_MCAST_CNT                                    0
#define BITL_MAC_P2_RX_MCAST_CNT_P2_RX_MCAST_CNT                                    32
#define BITM_MAC_P2_RX_MCAST_CNT_P2_RX_MCAST_CNT                                    0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_UCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_UCAST_CNT_P2_RX_UCAST_CNT                                    0
#define BITL_MAC_P2_RX_UCAST_CNT_P2_RX_UCAST_CNT                                    32
#define BITM_MAC_P2_RX_UCAST_CNT_P2_RX_UCAST_CNT                                    0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_CRC_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_CRC_ERR_CNT_P2_RX_CRC_ERR_CNT                                0
#define BITL_MAC_P2_RX_CRC_ERR_CNT_P2_RX_CRC_ERR_CNT                                32
#define BITM_MAC_P2_RX_CRC_ERR_CNT_P2_RX_CRC_ERR_CNT                                0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_ALGN_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_ALGN_ERR_CNT_P2_RX_ALGN_ERR_CNT                              0
#define BITL_MAC_P2_RX_ALGN_ERR_CNT_P2_RX_ALGN_ERR_CNT                              32
#define BITM_MAC_P2_RX_ALGN_ERR_CNT_P2_RX_ALGN_ERR_CNT                              0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_LS_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_LS_ERR_CNT_P2_RX_LS_ERR_CNT                                  0
#define BITL_MAC_P2_RX_LS_ERR_CNT_P2_RX_LS_ERR_CNT                                  32
#define BITM_MAC_P2_RX_LS_ERR_CNT_P2_RX_LS_ERR_CNT                                  0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_PHY_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_PHY_ERR_CNT_P2_RX_PHY_ERR_CNT                                0
#define BITL_MAC_P2_RX_PHY_ERR_CNT_P2_RX_PHY_ERR_CNT                                32
#define BITM_MAC_P2_RX_PHY_ERR_CNT_P2_RX_PHY_ERR_CNT                                0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TX_FRM_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TX_FRM_CNT_P2_TX_FRM_CNT                                        0
#define BITL_MAC_P2_TX_FRM_CNT_P2_TX_FRM_CNT                                        32
#define BITM_MAC_P2_TX_FRM_CNT_P2_TX_FRM_CNT                                        0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TX_BCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TX_BCAST_CNT_P2_TX_BCAST_CNT                                    0
#define BITL_MAC_P2_TX_BCAST_CNT_P2_TX_BCAST_CNT                                    32
#define BITM_MAC_P2_TX_BCAST_CNT_P2_TX_BCAST_CNT                                    0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TX_MCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TX_MCAST_CNT_P2_TX_MCAST_CNT                                    0
#define BITL_MAC_P2_TX_MCAST_CNT_P2_TX_MCAST_CNT                                    32
#define BITM_MAC_P2_TX_MCAST_CNT_P2_TX_MCAST_CNT                                    0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TX_UCAST_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TX_UCAST_CNT_P2_TX_UCAST_CNT                                    0
#define BITL_MAC_P2_TX_UCAST_CNT_P2_TX_UCAST_CNT                                    32
#define BITM_MAC_P2_TX_UCAST_CNT_P2_TX_UCAST_CNT                                    0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_DROP_FULL_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_DROP_FULL_CNT_P2_RX_DROP_FULL_CNT                            0
#define BITL_MAC_P2_RX_DROP_FULL_CNT_P2_RX_DROP_FULL_CNT                            32
#define BITM_MAC_P2_RX_DROP_FULL_CNT_P2_RX_DROP_FULL_CNT                            0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_DROP_FILT_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_DROP_FILT_CNT_P2_RX_DROP_FILT_CNT                            0
#define BITL_MAC_P2_RX_DROP_FILT_CNT_P2_RX_DROP_FILT_CNT                            32
#define BITM_MAC_P2_RX_DROP_FILT_CNT_P2_RX_DROP_FILT_CNT                            0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_IFG_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_IFG_ERR_CNT_P2_RX_IFG_ERR_CNT                                0
#define BITL_MAC_P2_RX_IFG_ERR_CNT_P2_RX_IFG_ERR_CNT                                32
#define BITM_MAC_P2_RX_IFG_ERR_CNT_P2_RX_IFG_ERR_CNT                                0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TX_IFG
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TX_IFG_P2_TX_IFG                                                0
#define BITL_MAC_P2_TX_IFG_P2_TX_IFG                                                8
#define BITM_MAC_P2_TX_IFG_P2_TX_IFG                                                0x000000FF

/* ----------------------------------------------------------------------------------------------------
          P2_LOOP
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_LOOP_P2_LOOPBACK_EN                                             0
#define BITL_MAC_P2_LOOP_P2_LOOPBACK_EN                                             1
#define BITM_MAC_P2_LOOP_P2_LOOPBACK_EN                                             0x00000001

#define ENUM_MAC_P2_LOOP_P2_LOOPBACK_EN_LOOPB_DIS                                   0x00000000 /* Normal Operation - Loopback Disabled. */
#define ENUM_MAC_P2_LOOP_P2_LOOPBACK_EN_LOOPB_EN                                    0x00000001 /* Loopback Enabled. */

/* ----------------------------------------------------------------------------------------------------
          P2_RX_CRC_EN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_CRC_EN_P2_CRC_CHK_EN                                         0
#define BITL_MAC_P2_RX_CRC_EN_P2_CRC_CHK_EN                                         1
#define BITM_MAC_P2_RX_CRC_EN_P2_CRC_CHK_EN                                         0x00000001

/* ----------------------------------------------------------------------------------------------------
          P2_RX_IFG
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_IFG_P2_RX_IFG                                                0
#define BITL_MAC_P2_RX_IFG_P2_RX_IFG                                                6
#define BITM_MAC_P2_RX_IFG_P2_RX_IFG                                                0x0000003F

/* ----------------------------------------------------------------------------------------------------
          P2_RX_MAX_LEN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_MAX_LEN_P2_RX_MAX_LEN                                        0
#define BITL_MAC_P2_RX_MAX_LEN_P2_RX_MAX_LEN                                        16
#define BITM_MAC_P2_RX_MAX_LEN_P2_RX_MAX_LEN                                        0x0000FFFF

/* ----------------------------------------------------------------------------------------------------
          P2_RX_MIN_LEN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_RX_MIN_LEN_P2_RX_MIN_LEN                                        0
#define BITL_MAC_P2_RX_MIN_LEN_P2_RX_MIN_LEN                                        16
#define BITM_MAC_P2_RX_MIN_LEN_P2_RX_MIN_LEN                                        0x0000FFFF

/* ----------------------------------------------------------------------------------------------------
          P2_LO_RFC
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_LO_RFC_P2_LO_RFC                                                0
#define BITL_MAC_P2_LO_RFC_P2_LO_RFC                                                9
#define BITM_MAC_P2_LO_RFC_P2_LO_RFC                                                0x000001FF

/* ----------------------------------------------------------------------------------------------------
          P2_HI_RFC
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_HI_RFC_P2_HI_RFC                                                0
#define BITL_MAC_P2_HI_RFC_P2_HI_RFC                                                9
#define BITM_MAC_P2_HI_RFC_P2_HI_RFC                                                0x000001FF

/* ----------------------------------------------------------------------------------------------------
          P2_LO_RXSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_LO_RXSIZE_P2_LO_RXSIZE                                          0
#define BITL_MAC_P2_LO_RXSIZE_P2_LO_RXSIZE                                          14
#define BITM_MAC_P2_LO_RXSIZE_P2_LO_RXSIZE                                          0x00003FFF

/* ----------------------------------------------------------------------------------------------------
          P2_HI_RXSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_HI_RXSIZE_P2_HI_RXSIZE                                          0
#define BITL_MAC_P2_HI_RXSIZE_P2_HI_RXSIZE                                          14
#define BITM_MAC_P2_HI_RXSIZE_P2_HI_RXSIZE                                          0x00003FFF

/* ----------------------------------------------------------------------------------------------------
          P2TOP1_TXSIZE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2TOP1_TXSIZE_P2TOP1_TXSIZE                                        0
#define BITL_MAC_P2TOP1_TXSIZE_P2TOP1_TXSIZE                                        14
#define BITM_MAC_P2TOP1_TXSIZE_P2TOP1_TXSIZE                                        0x00003FFF

/* ----------------------------------------------------------------------------------------------------
          P2_TTSCAH
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TTSCAH_P2_TTSCH_A                                               0
#define BITL_MAC_P2_TTSCAH_P2_TTSCH_A                                               32
#define BITM_MAC_P2_TTSCAH_P2_TTSCH_A                                               0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TTSCAL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TTSCAL_P2_TTSCL_A                                               0
#define BITL_MAC_P2_TTSCAL_P2_TTSCL_A                                               32
#define BITM_MAC_P2_TTSCAL_P2_TTSCL_A                                               0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TTSCBH
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TTSCBH_P2_TTSCH_B                                               0
#define BITL_MAC_P2_TTSCBH_P2_TTSCH_B                                               32
#define BITM_MAC_P2_TTSCBH_P2_TTSCH_B                                               0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TTSCBL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TTSCBL_P2_TTSCL_B                                               0
#define BITL_MAC_P2_TTSCBL_P2_TTSCL_B                                               32
#define BITM_MAC_P2_TTSCBL_P2_TTSCL_B                                               0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TTSCCH
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TTSCCH_P2_TTSCH_C                                               0
#define BITL_MAC_P2_TTSCCH_P2_TTSCH_C                                               32
#define BITM_MAC_P2_TTSCCH_P2_TTSCH_C                                               0xFFFFFFFF

/* ----------------------------------------------------------------------------------------------------
          P2_TTSCCL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_P2_TTSCCL_P2_TTSCL_C                                               0
#define BITL_MAC_P2_TTSCCL_P2_TTSCL_C                                               32
#define BITM_MAC_P2_TTSCCL_P2_TTSCL_C                                               0xFFFFFFFF

/* ====================================================================================================
        ADIN2111 Module Register Address Offset Definitions
   ==================================================================================================== */
#define ADIN2111_PMA_PMD_CNTRL1                                                     0x010000 /* PMA/PMD Control 1 Register */
#define ADIN2111_PMA_PMD_STAT1                                                      0x010001 /* PMA/PMD Status 1 Register */
#define ADIN2111_PMA_PMD_DEVS_IN_PKG1                                               0x010005 /* PMA/PMD MMD Devices in Package 1 */
#define ADIN2111_PMA_PMD_DEVS_IN_PKG2                                               0x010006 /* PMA/PMD MMD Devices in Package 2 Register */
#define ADIN2111_PMA_PMD_CNTRL2                                                     0x010007 /* PMA/PMD Control 2 Register */
#define ADIN2111_PMA_PMD_STAT2                                                      0x010008 /* PMA/PMD Status 2 */
#define ADIN2111_PMA_PMD_TX_DIS                                                     0x010009 /* PMA/PMD Transmit Disable Register */
#define ADIN2111_PMA_PMD_EXT_ABILITY                                                0x01000B /* PMA/PMD Extended Abilities Register */
#define ADIN2111_PMA_PMD_BT1_ABILITY                                                0x010012 /* BASE-T1 PMA/PMD Extended Ability Register */
#define ADIN2111_PMA_PMD_BT1_CONTROL                                                0x010834 /* BASE-T1 PMA/PMD Control Register */
#define ADIN2111_B10L_PMA_CNTRL                                                     0x0108F6 /* 10BASE-T1L PMA Control Register */
#define ADIN2111_B10L_PMA_STAT                                                      0x0108F7 /* 10BASE-T1L PMA Status Register */
#define ADIN2111_B10L_TEST_MODE_CNTRL                                               0x0108F8 /* 10BASE-T1L Test Mode Control Register */
#define ADIN2111_CR_STBL_CHK_FOFFS_SAT_THR                                          0x018015 /* Frequency Offset Saturation Threshold for CR Stability Check Register */
#define ADIN2111_SLV_FLTR_ECHO_ACQ_CR_KP                                            0x0181E7 /* Slave IIR Filter Change Echo Acquisition Clock Recovery Proportional Gain Register */
#define ADIN2111_B10L_PMA_LINK_STAT                                                 0x018302 /* 10BASE-T1L PMA Link Status Register */
#define ADIN2111_MSE_VAL                                                            0x01830B /* MSE Value Register */
#define ADIN2111_PCS_CNTRL1                                                         0x030000 /* PCS Control 1 Register */
#define ADIN2111_PCS_STAT1                                                          0x030001 /* PCS Status 1 Register */
#define ADIN2111_PCS_DEVS_IN_PKG1                                                   0x030005 /* PCS MMD Devices in Package 1 Register */
#define ADIN2111_PCS_DEVS_IN_PKG2                                                   0x030006 /* PCS MMD Devices in Package 2 Register */
#define ADIN2111_PCS_STAT2                                                          0x030008 /* PCS Status 2 Register */
#define ADIN2111_B10L_PCS_CNTRL                                                     0x0308E6 /* 10BASE-T1L PCS Control Register */
#define ADIN2111_B10L_PCS_STAT                                                      0x0308E7 /* 10BASE-T1L PCS Status Register */
#define ADIN2111_AN_DEVS_IN_PKG1                                                    0x070005 /* AUTO-_NEGOTIATION MMD Devices in Package 1 Register */
#define ADIN2111_AN_DEVS_IN_PKG2                                                    0x070006 /* AUTO-_NEGOTIATION MMD Devices in Package 2 Register */
#define ADIN2111_AN_CONTROL                                                         0x070200 /* BASE-T1 Autonegotiation Control Register */
#define ADIN2111_AN_STATUS                                                          0x070201 /* BASE-T1 Autonegotiation Status Register */
#define ADIN2111_AN_ADV_ABILITY_L                                                   0x070202 /* BASE-T1 Autonegotiation Advertisement [15:0] Register */
#define ADIN2111_AN_ADV_ABILITY_M                                                   0x070203 /* BASE-T1 Autonegotiation Advertisement [31:16] Register */
#define ADIN2111_AN_ADV_ABILITY_H                                                   0x070204 /* BASE-T1 Autonegotiation Advertisement [47:32] Register */
#define ADIN2111_AN_LP_ADV_ABILITY_L                                                0x070205 /* BASE-T1 Autonegotiation Link Partner Base Page Ability [15:0] Register */
#define ADIN2111_AN_LP_ADV_ABILITY_M                                                0x070206 /* BASE-T1 Autonegotiation Link Partner Base Page Ability [31:16] Register */
#define ADIN2111_AN_LP_ADV_ABILITY_H                                                0x070207 /* BASE-T1 Autonegotiation Link Partner Base Page Ability [47:32] Register */
#define ADIN2111_AN_NEXT_PAGE_L                                                     0x070208 /* BASE-T1 Autonegotiation Next Page Transmit [15:0] Register */
#define ADIN2111_AN_NEXT_PAGE_M                                                     0x070209 /* BASE-T1 Autonegotiation Next Page Transmit [31:16] Register */
#define ADIN2111_AN_NEXT_PAGE_H                                                     0x07020A /* BASE-T1 Autonegotiation Next Page Transmit [47:32] Register */
#define ADIN2111_AN_LP_NEXT_PAGE_L                                                  0x07020B /* BASE-T1 Autonegotiation Link Partner Next Page Ability [15:0] Register */
#define ADIN2111_AN_LP_NEXT_PAGE_M                                                  0x07020C /* BASE-T1 Autonegotiation Link Partner Next Page Ability [31:16] Register */
#define ADIN2111_AN_LP_NEXT_PAGE_H                                                  0x07020D /* BASE-T1 Autonegotiation Link Partner Next Page Ability [47:32] Register */
#define ADIN2111_AN_B10_ADV_ABILITY                                                 0x07020E /* 10BASE-T1 Autonegotiation Control Register */
#define ADIN2111_AN_B10_LP_ADV_ABILITY                                              0x07020F /* 10BASE-T1 Autonegotiation Status Register */
#define ADIN2111_AN_FRC_MODE_EN                                                     0x078000 /* Autonegotiation Force Mode Enable Register */
#define ADIN2111_AN_STATUS_EXTRA                                                    0x078001 /* Extra Autonegotiation Status Register */
#define ADIN2111_AN_PHY_INST_STATUS                                                 0x078030 /* PHY Instantaneous Status. */
#define ADIN2111_MMD1_DEV_ID1                                                       0x1E0002 /* Vendor Specific MMD 1 Device Identifier High Register */
#define ADIN2111_MMD1_DEV_ID2                                                       0x1E0003 /* Vendor Specific MMD 1 Device Identifier Low Register */
#define ADIN2111_MMD1_DEVS_IN_PKG1                                                  0x1E0005 /* Vendor Specific 1 MMD Devices in Package Register */
#define ADIN2111_MMD1_DEVS_IN_PKG2                                                  0x1E0006 /* Vendor Specific 1 MMD Devices in Package Register */
#define ADIN2111_MMD1_STATUS                                                        0x1E0008 /* Vendor Specific MMD 1 Status Register */
#define ADIN2111_CRSM_IRQ_STATUS                                                    0x1E0010 /* System Interrupt Status Register */
#define ADIN2111_CRSM_IRQ_MASK                                                      0x1E0020 /* System Interrupt Mask Register */
#define ADIN2111_CRSM_SFT_RST                                                       0x1E8810 /* Software Reset Register */
#define ADIN2111_CRSM_SFT_PD_CNTRL                                                  0x1E8812 /* Software Power-down Control Register */
#define ADIN2111_CRSM_PHY_SUBSYS_RST                                                0x1E8814 /* PHY Subsystem Reset Register */
#define ADIN2111_CRSM_MAC_IF_RST                                                    0x1E8815 /* PHY MAC Interface Reset Register */
#define ADIN2111_CRSM_STAT                                                          0x1E8818 /* System Status Register */
#define ADIN2111_CRSM_PMG_CNTRL                                                     0x1E8819 /* CRSM Power Management Control Register. */
#define ADIN2111_CRSM_DIAG_CLK_CTRL                                                 0x1E882C /* CRSM Diagnostics Clock Control. */
#define ADIN2111_MGMT_PRT_PKG                                                       0x1E8C22 /* Package Configuration Values Register */
#define ADIN2111_MGMT_MDIO_CNTRL                                                    0x1E8C30 /* MDIO Control Register */
#define ADIN2111_DIGIO_PINMUX                                                       0x1E8C56 /* Pinmux Configuration 1 Register */
#define ADIN2111_LED0_BLINK_TIME_CNTRL                                              0x1E8C80 /* LED 0 ON/_OFF Blink Time Register */
#define ADIN2111_LED1_BLINK_TIME_CNTRL                                              0x1E8C81 /* LED 1 ON/_OFF Blink Time Register */
#define ADIN2111_LED_CNTRL                                                          0x1E8C82 /* LED Control Register */
#define ADIN2111_LED_POLARITY                                                       0x1E8C83 /* LED Polarity Register */
#define ADIN2111_MMD2_DEV_ID1                                                       0x1F0002 /* Vendor Specific MMD 2 Device Identifier High Register. */
#define ADIN2111_MMD2_DEV_ID2                                                       0x1F0003 /* Vendor Specific MMD 2 Device Identifier Low Register. */
#define ADIN2111_MMD2_DEVS_IN_PKG1                                                  0x1F0005 /* Vendor Specific 2 MMD Devices in Package Register */
#define ADIN2111_MMD2_DEVS_IN_PKG2                                                  0x1F0006 /* Vendor Specific 2 MMD Devices in Package Register */
#define ADIN2111_MMD2_STATUS                                                        0x1F0008 /* Vendor Specific MMD 2 Status Register */
#define ADIN2111_PHY_SUBSYS_IRQ_STATUS                                              0x1F0011 /* PHY Subsystem Interrupt Status Register */
#define ADIN2111_PHY_SUBSYS_IRQ_MASK                                                0x1F0021 /* PHY Subsystem Interrupt Mask Register */
#define ADIN2111_FC_EN                                                              0x1F8001 /* Frame Checker Enable Register */
#define ADIN2111_FC_IRQ_EN                                                          0x1F8004 /* Frame Checker Interrupt Enable Register */
#define ADIN2111_FC_TX_SEL                                                          0x1F8005 /* Frame Checker Transmit Select Register */
#define ADIN2111_RX_ERR_CNT                                                         0x1F8008 /* Receive Error Count Register */
#define ADIN2111_FC_FRM_CNT_H                                                       0x1F8009 /* Frame Checker Count High Register */
#define ADIN2111_FC_FRM_CNT_L                                                       0x1F800A /* Frame Checker Count Low Register */
#define ADIN2111_FC_LEN_ERR_CNT                                                     0x1F800B /* Frame Checker Length Error Count Register */
#define ADIN2111_FC_ALGN_ERR_CNT                                                    0x1F800C /* Frame Checker Alignment Error Count Register */
#define ADIN2111_FC_SYMB_ERR_CNT                                                    0x1F800D /* Frame Checker Symbol Error Count Register */
#define ADIN2111_FC_OSZ_CNT                                                         0x1F800E /* Frame Checker Oversized Frame Count Register */
#define ADIN2111_FC_USZ_CNT                                                         0x1F800F /* Frame Checker Undersized Frame Count Register */
#define ADIN2111_FC_ODD_CNT                                                         0x1F8010 /* Frame Checker Odd Nibble Frame Count Register */
#define ADIN2111_FC_ODD_PRE_CNT                                                     0x1F8011 /* Frame Checker Odd Preamble Packet Count Register */
#define ADIN2111_FC_FALSE_CARRIER_CNT                                               0x1F8013 /* Frame Checker False Carrier Count Register */
#define ADIN2111_FG_EN                                                              0x1F8020 /* Frame Generator Enable Register */
#define ADIN2111_FG_CNTRL_RSTRT                                                     0x1F8021 /* Frame Generator CONTROL/_RESTART Register */
#define ADIN2111_FG_CONT_MODE_EN                                                    0x1F8022 /* Frame Generator Continuous Mode Enable Register */
#define ADIN2111_FG_IRQ_EN                                                          0x1F8023 /* Frame Generator Interrupt Enable Register */
#define ADIN2111_FG_FRM_LEN                                                         0x1F8025 /* Frame Generator Frame Length Register */
#define ADIN2111_FG_IFG_LEN                                                         0x1F8026 /* Frame Generator INTER-_FRAME Gap Length Register */
#define ADIN2111_FG_NFRM_H                                                          0x1F8027 /* Frame Generator Number of Frames High Register */
#define ADIN2111_FG_NFRM_L                                                          0x1F8028 /* Frame Generator Number of Frames Low Register */
#define ADIN2111_FG_DONE                                                            0x1F8029 /* Frame Generator Done Register */
#define ADIN2111_MAC_IF_LOOPBACK                                                    0x1F8055 /* MAC Interface Loopbacks Configuration Register */
#define ADIN2111_MAC_IF_SOP_CNTRL                                                   0x1F805A /* MAC Start Of Packet (SOP) Generation Control Register */

/* ====================================================================================================
        ADIN2111 Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_CNTRL1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_CNTRL1_LB_PMA_LOC_EN                                           0 /* Enables PMA Local Loopback. */
#define BITL_PMA_PMD_CNTRL1_LB_PMA_LOC_EN                                           1 /* Enables PMA Local Loopback. */
#define BITM_PMA_PMD_CNTRL1_LB_PMA_LOC_EN                                           0x0001 /* Enables PMA Local Loopback. */
#define BITP_PMA_PMD_CNTRL1_PMA_SFT_PD                                              11 /* PMA Software Power-down. */
#define BITL_PMA_PMD_CNTRL1_PMA_SFT_PD                                              1 /* PMA Software Power-down. */
#define BITM_PMA_PMD_CNTRL1_PMA_SFT_PD                                              0x0800 /* PMA Software Power-down. */
#define BITP_PMA_PMD_CNTRL1_PMA_SFT_RST                                             15 /* PMA Software Reset. */
#define BITL_PMA_PMD_CNTRL1_PMA_SFT_RST                                             1 /* PMA Software Reset. */
#define BITM_PMA_PMD_CNTRL1_PMA_SFT_RST                                             0x8000 /* PMA Software Reset. */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_STAT1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_STAT1_PMA_SFT_PD_ABLE                                          1 /* PMA Software Powerdown Able */
#define BITL_PMA_PMD_STAT1_PMA_SFT_PD_ABLE                                          1 /* PMA Software Powerdown Able */
#define BITM_PMA_PMD_STAT1_PMA_SFT_PD_ABLE                                          0x0002 /* PMA Software Powerdown Able */
#define BITP_PMA_PMD_STAT1_PMA_LINK_STAT_OK_LL                                      2 /* PMA Link Status. */
#define BITL_PMA_PMD_STAT1_PMA_LINK_STAT_OK_LL                                      1 /* PMA Link Status. */
#define BITM_PMA_PMD_STAT1_PMA_LINK_STAT_OK_LL                                      0x0004 /* PMA Link Status. */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_DEVS_IN_PKG1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_DEVS_IN_PKG1_PMA_PMD_DEVS_IN_PKG1                              0 /* PMA/PMD MMD Devices in Package */
#define BITL_PMA_PMD_DEVS_IN_PKG1_PMA_PMD_DEVS_IN_PKG1                              16 /* PMA/PMD MMD Devices in Package */
#define BITM_PMA_PMD_DEVS_IN_PKG1_PMA_PMD_DEVS_IN_PKG1                              0xFFFF /* PMA/PMD MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_DEVS_IN_PKG2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_DEVS_IN_PKG2_PMA_PMD_DEVS_IN_PKG2                              0 /* PMA/PMD MMD Devices in Package */
#define BITL_PMA_PMD_DEVS_IN_PKG2_PMA_PMD_DEVS_IN_PKG2                              16 /* PMA/PMD MMD Devices in Package */
#define BITM_PMA_PMD_DEVS_IN_PKG2_PMA_PMD_DEVS_IN_PKG2                              0xFFFF /* PMA/PMD MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_CNTRL2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL                                        0 /* PMA/PMD Type Selection */
#define BITL_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL                                        7 /* PMA/PMD Type Selection */
#define BITM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL                                        0x007F /* PMA/PMD Type Selection */

#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_CX4_PMA_PMD                 0x0000 /* TS_10GBASE_CX4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_EW_PMA_PMD                  0x0001 /* TS_10GBASE_EW_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_LW_PMA_PMD                  0x0002 /* TS_10GBASE_LW_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_SW_PMA_PMD                  0x0003 /* TS_10GBASE_SW_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_LX4_PMA_PMD                 0x0004 /* TS_10GBASE_LX4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_ER_PMA_PMD                  0x0005 /* TS_10GBASE_ER_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_LR_PMA_PMD                  0x0006 /* TS_10GBASE_LR_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_SR_PMA_PMD                  0x0007 /* TS_10GBASE_SR_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_LRM_PMA_PMD                 0x0008 /* TS_10GBASE_LRM_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_T_PMA                       0x0009 /* TS_10GBASE_T_PMA */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_KX4_PMA_PMD                 0x000A /* TS_10GBASE_KX4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_KR_PMA_PMD                  0x000B /* TS_10GBASE_KR_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_1000BASE_T_PMA_PMD                  0x000C /* TS_1000BASE_T_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_1000BASE_KX_PMA_PMD                 0x000D /* TS_1000BASE_KX_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_100BASE_TX_PMA_PMD                  0x000E /* TS_100BASE_TX_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10BASE_T_PMA_PMD                    0x000F /* TS_10BASE_T_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10_1GBASE_PRX_D1                    0x0010 /* TS_10_1GBASE_PRX_D1 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10_1GBASE_PRX_D2                    0x0011 /* TS_10_1GBASE_PRX_D2 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10_1GBASE_PRX_D3                    0x0012 /* TS_10_1GBASE_PRX_D3 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_PR_D1                       0x0013 /* TS_10GBASE_PR_D1 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_PR_D2                       0x0014 /* TS_10GBASE_PR_D2 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_PR_D3                       0x0015 /* TS_10GBASE_PR_D3 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10_1GBASE_PRX_U1                    0x0016 /* TS_10_1GBASE_PRX_U1 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10_1GBASE_PRX_U2                    0x0017 /* TS_10_1GBASE_PRX_U2 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10_1GBASE_PRX_U3                    0x0018 /* TS_10_1GBASE_PRX_U3 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_PR_U1                       0x0019 /* TS_10GBASE_PR_U1 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_PR_U3                       0x001A /* TS_10GBASE_PR_U3 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_RESERVED                            0x001B /* TS_RESERVED */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_PR_D4                       0x001C /* TS_10GBASE_PR_D4 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10_1GBASE_PRX_D4                    0x001D /* TS_10_1GBASE_PRX_D4 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GBASE_PR_U4                       0x001E /* TS_10GBASE_PR_U4 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10_1GBASE_PRX_U4                    0x001F /* TS_10_1GBASE_PRX_U4 */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_40GBASE_KR4_PMA_PMD                 0x0020 /* TS_40GBASE_KR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_40GBASE_CR4_PMA_PMD                 0x0021 /* TS_40GBASE_CR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_40GBASE_SR4_PMA_PMD                 0x0022 /* TS_40GBASE_SR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_40GBASE_LR4_PMA_PMD                 0x0023 /* TS_40GBASE_LR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_40GBASE_FR_PMA_PMD                  0x0024 /* TS_40GBASE_FR_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_40GBASE_ER4_PMA_PMD                 0x0025 /* TS_40GBASE_ER4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_40GBASE_T_PMA                       0x0026 /* TS_40GBASE_T_PMA */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_100GBASE_CR10_PMA_PMD               0x0028 /* TS_100GBASE_CR10_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_100GBASE_SR10_PMA_PMD               0x0029 /* TS_100GBASE_SR10_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_100GBASE_LR4_PMA_PMD                0x002A /* TS_100GBASE_LR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_100GBASE_ER4_PMA_PMD                0x002B /* TS_100GBASE_ER4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_100GBASE_KP4_PMA_PMD                0x002C /* TS_100GBASE_KP4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_100GBASE_KR4_PMA_PMD                0x002D /* TS_100GBASE_KR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_100GBASE_CR4_PMA_PMD                0x002E /* TS_100GBASE_CR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_100GBASE_SR4_PMA_PMD                0x002F /* TS_100GBASE_SR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_2_5GBASE_T_PMA                      0x0030 /* TS_2_5GBASE_T_PMA */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_5GBASE_T_PMA                        0x0031 /* TS_5GBASE_T_PMA */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GPASS_XR_D_PMA_PMD                0x0032 /* TS_10GPASS_XR_D_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_10GPASS_XR_U_PMA_PMD                0x0033 /* TS_10GPASS_XR_U_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_BASE_H_PMA_PMD                      0x0034 /* TS_BASE_H_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_25GBASE_LR_PMA_PMD                  0x0035 /* TS_25GBASE_LR_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_25GBASE_ER_PMA_PMD                  0x0036 /* TS_25GBASE_ER_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_25GBASE_T_PMA                       0x0037 /* TS_25GBASE_T_PMA */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_25GBASE_CR_OR_25GBASE_CR_S_PMA_PMD  0x0038 /* TS_25GBASE_CR_OR_25GBASE_CR_S_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_25GBASE_KR_OR_25GBASE_KR_S_PMA_PMD  0x0039 /* TS_25GBASE_KR_OR_25GBASE_KR_S_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_25GBASE_SR_PMA_PMD                  0x003A /* TS_25GBASE_SR_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_BASE_T1_PMA_PMD                     0x003D /* TS_BASE_T1_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_200GBASE_DR4_PMA_PMD                0x0053 /* TS_200GBASE_DR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_200GBASE_FR4_PMA_PMD                0x0054 /* TS_200GBASE_FR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_200GBASE_LR4_PMA_PMD                0x0055 /* TS_200GBASE_LR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_400GBASE_SR16_PMA_PMD               0x0059 /* TS_400GBASE_SR16_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_400GBASE_DR4_PMA_PMD                0x005A /* TS_400GBASE_DR4_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_400GBASE_FR8_PMA_PMD                0x005B /* TS_400GBASE_FR8_PMA_PMD */
#define ENUM_PMA_PMD_CNTRL2_PMA_PMD_TYPE_SEL_TS_400GBASE_LR8_PMA_PMD                0x005C /* TS_400GBASE_LR8_PMA_PMD */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_STAT2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_STAT2_LB_PMA_LOC_ABLE                                          0 /* PMA Local Loopback Able */
#define BITL_PMA_PMD_STAT2_LB_PMA_LOC_ABLE                                          1 /* PMA Local Loopback Able */
#define BITM_PMA_PMD_STAT2_LB_PMA_LOC_ABLE                                          0x0001 /* PMA Local Loopback Able */
#define BITP_PMA_PMD_STAT2_PMA_PMD_TX_DIS_ABLE                                      8 /* PMA/PMD Tx Disable */
#define BITL_PMA_PMD_STAT2_PMA_PMD_TX_DIS_ABLE                                      1 /* PMA/PMD Tx Disable */
#define BITM_PMA_PMD_STAT2_PMA_PMD_TX_DIS_ABLE                                      0x0100 /* PMA/PMD Tx Disable */
#define BITP_PMA_PMD_STAT2_PMA_PMD_EXT_ABLE                                         9 /* PHY Extended Abilities Support */
#define BITL_PMA_PMD_STAT2_PMA_PMD_EXT_ABLE                                         1 /* PHY Extended Abilities Support */
#define BITM_PMA_PMD_STAT2_PMA_PMD_EXT_ABLE                                         0x0200 /* PHY Extended Abilities Support */
#define BITP_PMA_PMD_STAT2_PMA_PMD_PRESENT                                          14 /* PMA/PMD Present */
#define BITL_PMA_PMD_STAT2_PMA_PMD_PRESENT                                          2 /* PMA/PMD Present */
#define BITM_PMA_PMD_STAT2_PMA_PMD_PRESENT                                          0xC000 /* PMA/PMD Present */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_TX_DIS
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_TX_DIS_PMA_TX_DIS                                              0 /* PMD Transmit Disable. */
#define BITL_PMA_PMD_TX_DIS_PMA_TX_DIS                                              1 /* PMD Transmit Disable. */
#define BITM_PMA_PMD_TX_DIS_PMA_TX_DIS                                              0x0001 /* PMD Transmit Disable. */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_EXT_ABILITY
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_EXT_ABILITY_PMA_PMD_BT1_ABLE                                   11 /* PHY Supports BASE-T1 */
#define BITL_PMA_PMD_EXT_ABILITY_PMA_PMD_BT1_ABLE                                   1 /* PHY Supports BASE-T1 */
#define BITM_PMA_PMD_EXT_ABILITY_PMA_PMD_BT1_ABLE                                   0x0800 /* PHY Supports BASE-T1 */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_BT1_ABILITY
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_BT1_ABILITY_B100_ABILITY                                       0 /* 100BASE-T1 Ability. */
#define BITL_PMA_PMD_BT1_ABILITY_B100_ABILITY                                       1 /* 100BASE-T1 Ability. */
#define BITM_PMA_PMD_BT1_ABILITY_B100_ABILITY                                       0x0001 /* 100BASE-T1 Ability. */
#define BITP_PMA_PMD_BT1_ABILITY_B1000_ABILITY                                      1 /* 1000BASE-T1 Ability. */
#define BITL_PMA_PMD_BT1_ABILITY_B1000_ABILITY                                      1 /* 1000BASE-T1 Ability. */
#define BITM_PMA_PMD_BT1_ABILITY_B1000_ABILITY                                      0x0002 /* 1000BASE-T1 Ability. */
#define BITP_PMA_PMD_BT1_ABILITY_B10L_ABILITY                                       2 /* 10BASE-T1L Ability. */
#define BITL_PMA_PMD_BT1_ABILITY_B10L_ABILITY                                       1 /* 10BASE-T1L Ability. */
#define BITM_PMA_PMD_BT1_ABILITY_B10L_ABILITY                                       0x0004 /* 10BASE-T1L Ability. */
#define BITP_PMA_PMD_BT1_ABILITY_B10S_ABILITY                                       3 /* 10BASE-T1S Ability. */
#define BITL_PMA_PMD_BT1_ABILITY_B10S_ABILITY                                       1 /* 10BASE-T1S Ability. */
#define BITM_PMA_PMD_BT1_ABILITY_B10S_ABILITY                                       0x0008 /* 10BASE-T1S Ability. */

/* ----------------------------------------------------------------------------------------------------
          PMA_PMD_BT1_CONTROL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PMA_PMD_BT1_CONTROL_BT1_TYPE_SEL                                       0 /* BASE-T1 Type Selection */
#define BITL_PMA_PMD_BT1_CONTROL_BT1_TYPE_SEL                                       4 /* BASE-T1 Type Selection */
#define BITM_PMA_PMD_BT1_CONTROL_BT1_TYPE_SEL                                       0x000F /* BASE-T1 Type Selection */
#define BITP_PMA_PMD_BT1_CONTROL_CFG_MST                                            14 /* Master-slave Config. */
#define BITL_PMA_PMD_BT1_CONTROL_CFG_MST                                            1 /* Master-slave Config. */
#define BITM_PMA_PMD_BT1_CONTROL_CFG_MST                                            0x4000 /* Master-slave Config. */

#define ENUM_PMA_PMD_BT1_CONTROL_BT1_TYPE_SEL_TS_10BASE_T1L                         0x0002 /* 10BASE-T1L */

/* ----------------------------------------------------------------------------------------------------
          B10L_PMA_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_B10L_PMA_CNTRL_B10L_LB_PMA_LOC_EN                                      0 /* 10BASE-T1L PMA Loopback. */
#define BITL_B10L_PMA_CNTRL_B10L_LB_PMA_LOC_EN                                      1 /* 10BASE-T1L PMA Loopback. */
#define BITM_B10L_PMA_CNTRL_B10L_LB_PMA_LOC_EN                                      0x0001 /* 10BASE-T1L PMA Loopback. */
#define BITP_B10L_PMA_CNTRL_B10L_EEE                                                10 /* 10BASE-T1L EEE Enable. */
#define BITL_B10L_PMA_CNTRL_B10L_EEE                                                1 /* 10BASE-T1L EEE Enable. */
#define BITM_B10L_PMA_CNTRL_B10L_EEE                                                0x0400 /* 10BASE-T1L EEE Enable. */
#define BITP_B10L_PMA_CNTRL_B10L_TX_LVL_HI                                          12 /* 10BASE-T1L Transmit Voltage Amplitude Control. */
#define BITL_B10L_PMA_CNTRL_B10L_TX_LVL_HI                                          1 /* 10BASE-T1L Transmit Voltage Amplitude Control. */
#define BITM_B10L_PMA_CNTRL_B10L_TX_LVL_HI                                          0x1000 /* 10BASE-T1L Transmit Voltage Amplitude Control. */
#define BITP_B10L_PMA_CNTRL_B10L_TX_DIS_MODE_EN                                     14 /* 10BASE-T1L Transmit Disable Mode. */
#define BITL_B10L_PMA_CNTRL_B10L_TX_DIS_MODE_EN                                     1 /* 10BASE-T1L Transmit Disable Mode. */
#define BITM_B10L_PMA_CNTRL_B10L_TX_DIS_MODE_EN                                     0x4000 /* 10BASE-T1L Transmit Disable Mode. */

/* ----------------------------------------------------------------------------------------------------
          B10L_PMA_STAT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_B10L_PMA_STAT_B10L_EEE_ABLE                                            10 /* 10BASE-T1L EEE Ability */
#define BITL_B10L_PMA_STAT_B10L_EEE_ABLE                                            1 /* 10BASE-T1L EEE Ability */
#define BITM_B10L_PMA_STAT_B10L_EEE_ABLE                                            0x0400 /* 10BASE-T1L EEE Ability */
#define BITP_B10L_PMA_STAT_B10L_PMA_SFT_PD_ABLE                                     11/* PMA Supports Powerdown */
#define BITL_B10L_PMA_STAT_B10L_PMA_SFT_PD_ABLE                                     1 /* PMA Supports Powerdown */
#define BITM_B10L_PMA_STAT_B10L_PMA_SFT_PD_ABLE                                     0x0800 /* PMA Supports Powerdown */
#define BITP_B10L_PMA_STAT_B10L_TX_LVL_HI_ABLE                                      12 /* 10BASE-T1L High Voltage Tx Ability */
#define BITL_B10L_PMA_STAT_B10L_TX_LVL_HI_ABLE                                      1 /* 10BASE-T1L High Voltage Tx Ability */
#define BITM_B10L_PMA_STAT_B10L_TX_LVL_HI_ABLE                                      0x1000 /* 10BASE-T1L High Voltage Tx Ability */
#define BITP_B10L_PMA_STAT_B10L_LB_PMA_LOC_ABLE                                     13 /* 10BASE-T1L PMA Loopback Ability */
#define BITL_B10L_PMA_STAT_B10L_LB_PMA_LOC_ABLE                                     1 /* 10BASE-T1L PMA Loopback Ability */
#define BITM_B10L_PMA_STAT_B10L_LB_PMA_LOC_ABLE                                     0x2000 /* 10BASE-T1L PMA Loopback Ability */

/* ----------------------------------------------------------------------------------------------------
          B10L_TEST_MODE_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_B10L_TEST_MODE_CNTRL_B10L_TX_TEST_MODE                                 13 /* 10BASE-T1L Transmitter Test Mode. */
#define BITL_B10L_TEST_MODE_CNTRL_B10L_TX_TEST_MODE                                 3 /* 10BASE-T1L Transmitter Test Mode. */
#define BITM_B10L_TEST_MODE_CNTRL_B10L_TX_TEST_MODE                                 0xE000 /* 10BASE-T1L Transmitter Test Mode. */

#define ENUM_B10L_TEST_MODE_CNTRL_B10L_TX_TEST_MODE_IEEE_TX_TM_NONE                 0x0000 /* Normal operation. */
#define ENUM_B10L_TEST_MODE_CNTRL_B10L_TX_TEST_MODE_IEEE_TX_TM_JITTER               0x0001 /* Test mode 1 - Transmitter output voltage and timing jitter test mode.  When test mode 1 is enabled, the PHY shall repeatedly transmit the data symbol sequence (+1, -1). */
#define ENUM_B10L_TEST_MODE_CNTRL_B10L_TX_TEST_MODE_IEEE_TX_TM_DROOP                0x0002 /* Test mode 2 - Transmitter output droop test mode.  When test mode 2 is enabled, the PHY shall transmit ten "+1" symbols followed by ten "-1" symbols. */
#define ENUM_B10L_TEST_MODE_CNTRL_B10L_TX_TEST_MODE_IEEE_TX_TM_IDLE                 0x0003 /* Test mode 3 - Normal operation in Idle mode.  When test mode 3 is enabled, the PHY shall transmit as in non-test operation and in the MASTER data mode with data set to normal Inter-Frame idle signals. */

/* ----------------------------------------------------------------------------------------------------
          CR_STBL_CHK_FOFFS_SAT_THR
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CR_STBL_CHK_FOFFS_SAT_THR_CR_STBL_CHK_FOFFS_SAT_THR                    0 /* Frequency Offset Saturation Threshold for CR Stability Check. */
#define BITL_CR_STBL_CHK_FOFFS_SAT_THR_CR_STBL_CHK_FOFFS_SAT_THR                    11 /* Frequency Offset Saturation Threshold for CR Stability Check. */
#define BITM_CR_STBL_CHK_FOFFS_SAT_THR_CR_STBL_CHK_FOFFS_SAT_THR                    0x07FF /* Frequency Offset Saturation Threshold for CR Stability Check. */

/* ----------------------------------------------------------------------------------------------------
          SLV_FLTR_ECHO_ACQ_CR_KP
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SLV_FLTR_ECHO_ACQ_CR_KP_SLV_FLTR_ECHO_ACQ_CR_KP                        0 /* Slave IIR Filter Change Echo Acquisition Clock Recovery Proportional Gain. */
#define BITL_SLV_FLTR_ECHO_ACQ_CR_KP_SLV_FLTR_ECHO_ACQ_CR_KP                        16 /* Slave IIR Filter Change Echo Acquisition Clock Recovery Proportional Gain. */
#define BITM_SLV_FLTR_ECHO_ACQ_CR_KP_SLV_FLTR_ECHO_ACQ_CR_KP                        0xFFFF /* Slave IIR Filter Change Echo Acquisition Clock Recovery Proportional Gain. */

/* ----------------------------------------------------------------------------------------------------
          B10L_PMA_LINK_STAT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_B10L_PMA_LINK_STAT_B10L_LINK_STAT_OK                                   0 /* Link Status OK */
#define BITL_B10L_PMA_LINK_STAT_B10L_LINK_STAT_OK                                   1 /* Link Status OK */
#define BITM_B10L_PMA_LINK_STAT_B10L_LINK_STAT_OK                                   0x0001 /* Link Status OK */
#define BITP_B10L_PMA_LINK_STAT_B10L_LINK_STAT_OK_LL                                1 /* Link Status Ok Latch Low */
#define BITL_B10L_PMA_LINK_STAT_B10L_LINK_STAT_OK_LL                                1 /* Link Status Ok Latch Low */
#define BITM_B10L_PMA_LINK_STAT_B10L_LINK_STAT_OK_LL                                0x0002 /* Link Status Ok Latch Low */
#define BITP_B10L_PMA_LINK_STAT_B10L_DSCR_STAT_OK                                   4 /* 10BASE-T1L Descrambler Status Ok */
#define BITL_B10L_PMA_LINK_STAT_B10L_DSCR_STAT_OK                                   1 /* 10BASE-T1L Descrambler Status Ok */
#define BITM_B10L_PMA_LINK_STAT_B10L_DSCR_STAT_OK                                   0x0010 /* 10BASE-T1L Descrambler Status Ok */
#define BITP_B10L_PMA_LINK_STAT_B10L_DSCR_STAT_OK_LL                                5 /* BASE-T1L Descrambler Status Ok Latch Low */
#define BITL_B10L_PMA_LINK_STAT_B10L_DSCR_STAT_OK_LL                                1 /* BASE-T1L Descrambler Status Ok Latch Low */
#define BITM_B10L_PMA_LINK_STAT_B10L_DSCR_STAT_OK_LL                                0x0020 /* BASE-T1L Descrambler Status Ok Latch Low */
#define BITP_B10L_PMA_LINK_STAT_B10L_LOC_RCVR_STAT_OK                               6 /* 10BASE-T1L Local Receiver Status Ok */
#define BITL_B10L_PMA_LINK_STAT_B10L_LOC_RCVR_STAT_OK                               1 /* 10BASE-T1L Local Receiver Status Ok */
#define BITM_B10L_PMA_LINK_STAT_B10L_LOC_RCVR_STAT_OK                               0x0040 /* 10BASE-T1L Local Receiver Status Ok */
#define BITP_B10L_PMA_LINK_STAT_B10L_LOC_RCVR_STAT_OK_LL                            7 /* 10BASE-T1L Local Receiver Status Ok Latch Low */
#define BITL_B10L_PMA_LINK_STAT_B10L_LOC_RCVR_STAT_OK_LL                            1 /* 10BASE-T1L Local Receiver Status Ok Latch Low */
#define BITM_B10L_PMA_LINK_STAT_B10L_LOC_RCVR_STAT_OK_LL                            0x0080 /* 10BASE-T1L Local Receiver Status Ok Latch Low */
#define BITP_B10L_PMA_LINK_STAT_B10L_REM_RCVR_STAT_OK                               8 /* 10BASE-T1L Remote Receiver Status Ok */
#define BITL_B10L_PMA_LINK_STAT_B10L_REM_RCVR_STAT_OK                               1 /* 10BASE-T1L Remote Receiver Status Ok */
#define BITM_B10L_PMA_LINK_STAT_B10L_REM_RCVR_STAT_OK                               0x0100 /* 10BASE-T1L Remote Receiver Status Ok */
#define BITP_B10L_PMA_LINK_STAT_B10L_REM_RCVR_STAT_OK_LL                            9 /* 10BASE-T1L Remote Receiver Status Ok Latch Low */
#define BITL_B10L_PMA_LINK_STAT_B10L_REM_RCVR_STAT_OK_LL                            1 /* 10BASE-T1L Remote Receiver Status Ok Latch Low */
#define BITM_B10L_PMA_LINK_STAT_B10L_REM_RCVR_STAT_OK_LL                            0x0200 /* 10BASE-T1L Remote Receiver Status Ok Latch Low */

/* ----------------------------------------------------------------------------------------------------
          MSE_VAL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MSE_VAL_MSE_VAL                                                        0 /* MSE Value */
#define BITL_MSE_VAL_MSE_VAL                                                        16 /* MSE Value */
#define BITM_MSE_VAL_MSE_VAL                                                        0xFFFF /* MSE Value */

/* ----------------------------------------------------------------------------------------------------
          PCS_CNTRL1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCS_CNTRL1_PCS_SFT_PD                                                  11 /* PCS Software Power-down. */
#define BITL_PCS_CNTRL1_PCS_SFT_PD                                                  1 /* PCS Software Power-down. */
#define BITM_PCS_CNTRL1_PCS_SFT_PD                                                  0x0800 /* PCS Software Power-down. */
#define BITP_PCS_CNTRL1_LB_PCS_EN                                                   14 /* PCS Loopback Enable */
#define BITL_PCS_CNTRL1_LB_PCS_EN                                                   1 /* PCS Loopback Enable */
#define BITM_PCS_CNTRL1_LB_PCS_EN                                                   0x4000 /* PCS Loopback Enable */
#define BITP_PCS_CNTRL1_PCS_SFT_RST                                                 15 /* PCS Software Reset. */
#define BITL_PCS_CNTRL1_PCS_SFT_RST                                                 1 /* PCS Software Reset. */
#define BITM_PCS_CNTRL1_PCS_SFT_RST                                                 0x8000 /* PCS Software Reset. */

/* ----------------------------------------------------------------------------------------------------
          PCS_STAT1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCS_STAT1_PCS_SFT_PD_ABLE                                              1 /* PCS Software Powerdown Able */
#define BITL_PCS_STAT1_PCS_SFT_PD_ABLE                                              1 /* PCS Software Powerdown Able */
#define BITM_PCS_STAT1_PCS_SFT_PD_ABLE                                              0x0002 /* PCS Software Powerdown Able */

/* ----------------------------------------------------------------------------------------------------
          PCS_DEVS_IN_PKG1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCS_DEVS_IN_PKG1_PCS_DEVS_IN_PKG1                                      0 /* PCS MMD Devices in Package */
#define BITL_PCS_DEVS_IN_PKG1_PCS_DEVS_IN_PKG1                                      16 /* PCS MMD Devices in Package */
#define BITM_PCS_DEVS_IN_PKG1_PCS_DEVS_IN_PKG1                                      0xFFFF /* PCS MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          PCS_DEVS_IN_PKG2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCS_DEVS_IN_PKG2_PCS_DEVS_IN_PKG2                                      0 /* PCS MMD Devices in Package */
#define BITL_PCS_DEVS_IN_PKG2_PCS_DEVS_IN_PKG2                                      16 /* PCS MMD Devices in Package */
#define BITM_PCS_DEVS_IN_PKG2_PCS_DEVS_IN_PKG2                                      0xFFFF /* PCS MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          PCS_STAT2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCS_STAT2_PCS_PRESENT                                                  14 /* PCS Present */
#define BITL_PCS_STAT2_PCS_PRESENT                                                  2 /* PCS Present */
#define BITM_PCS_STAT2_PCS_PRESENT                                                  0xC000 /* PCS Present */

/* ----------------------------------------------------------------------------------------------------
          B10L_PCS_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_B10L_PCS_CNTRL_B10L_LB_PCS_EN                                          14 /* PCS Loopback Enable */
#define BITL_B10L_PCS_CNTRL_B10L_LB_PCS_EN                                          1 /* PCS Loopback Enable */
#define BITM_B10L_PCS_CNTRL_B10L_LB_PCS_EN                                          0x4000 /* PCS Loopback Enable */

/* ----------------------------------------------------------------------------------------------------
          B10L_PCS_STAT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_B10L_PCS_STAT_B10L_PCS_DSCR_STAT_OK_LL                                 2 /* PCS Descrambler Status. */
#define BITL_B10L_PCS_STAT_B10L_PCS_DSCR_STAT_OK_LL                                 1 /* PCS Descrambler Status. */
#define BITM_B10L_PCS_STAT_B10L_PCS_DSCR_STAT_OK_LL                                 0x0004 /* PCS Descrambler Status. */

/* ----------------------------------------------------------------------------------------------------
          AN_DEVS_IN_PKG1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_DEVS_IN_PKG1_AN_DEVS_IN_PKG1                                        0 /* Autonegotiation MMD Devices in Package */
#define BITL_AN_DEVS_IN_PKG1_AN_DEVS_IN_PKG1                                        16 /* Autonegotiation MMD Devices in Package */
#define BITM_AN_DEVS_IN_PKG1_AN_DEVS_IN_PKG1                                        0xFFFF /* Autonegotiation MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          AN_DEVS_IN_PKG2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_DEVS_IN_PKG2_AN_DEVS_IN_PKG2                                        0 /* Autonegotiation MMD Devices in Package */
#define BITL_AN_DEVS_IN_PKG2_AN_DEVS_IN_PKG2                                        16 /* Autonegotiation MMD Devices in Package */
#define BITM_AN_DEVS_IN_PKG2_AN_DEVS_IN_PKG2                                        0xFFFF /* Autonegotiation MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          AN_CONTROL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_CONTROL_AN_RESTART                                                  9 /* Autonegotiation Restart */
#define BITL_AN_CONTROL_AN_RESTART                                                  1 /* Autonegotiation Restart */
#define BITM_AN_CONTROL_AN_RESTART                                                  0x0200 /* Autonegotiation Restart */
#define BITP_AN_CONTROL_AN_EN                                                       12 /* Autonegotiation Enable. */
#define BITL_AN_CONTROL_AN_EN                                                       1 /* Autonegotiation Enable. */
#define BITM_AN_CONTROL_AN_EN                                                       0x1000 /* Autonegotiation Enable. */

/* ----------------------------------------------------------------------------------------------------
          AN_STATUS
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_STATUS_AN_LINK_STATUS                                               2 /* Link Status. */
#define BITL_AN_STATUS_AN_LINK_STATUS                                               1 /* Link Status. */
#define BITM_AN_STATUS_AN_LINK_STATUS                                               0x0004 /* Link Status. */
#define BITP_AN_STATUS_AN_ABLE                                                      3 /* Autonegotiation Ability. */
#define BITL_AN_STATUS_AN_ABLE                                                      1 /* Autonegotiation Ability. */
#define BITM_AN_STATUS_AN_ABLE                                                      0x0008 /* Autonegotiation Ability. */
#define BITP_AN_STATUS_AN_REMOTE_FAULT                                              4 /* Autonegotiation Remote Fault */
#define BITL_AN_STATUS_AN_REMOTE_FAULT                                              1 /* Autonegotiation Remote Fault */
#define BITM_AN_STATUS_AN_REMOTE_FAULT                                              0x0010 /* Autonegotiation Remote Fault */
#define BITP_AN_STATUS_AN_COMPLETE                                                  5 /* Autonegotiation Complete. */
#define BITL_AN_STATUS_AN_COMPLETE                                                  1 /* Autonegotiation Complete. */
#define BITM_AN_STATUS_AN_COMPLETE                                                  0x0020  /* Autonegotiation Complete. */
#define BITP_AN_STATUS_AN_PAGE_RX                                                   6 /* Page Received */
#define BITL_AN_STATUS_AN_PAGE_RX                                                   1 /* Page Received */
#define BITM_AN_STATUS_AN_PAGE_RX                                                   0x0040U /* Page Received */

/* ----------------------------------------------------------------------------------------------------
          AN_ADV_ABILITY_L
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_ADV_ABILITY_L_AN_ADV_SELECTOR                                       0 /* Selector. */
#define BITL_AN_ADV_ABILITY_L_AN_ADV_SELECTOR                                       5 /* Selector. */
#define BITM_AN_ADV_ABILITY_L_AN_ADV_SELECTOR                                       0x001F /* Selector. */
#define BITP_AN_ADV_ABILITY_L_AN_ADV_PAUSE                                          10 /* Pause Ability. */
#define BITL_AN_ADV_ABILITY_L_AN_ADV_PAUSE                                          2 /* Pause Ability. */
#define BITM_AN_ADV_ABILITY_L_AN_ADV_PAUSE                                          0x0C00 /* Pause Ability. */
#define BITP_AN_ADV_ABILITY_L_AN_ADV_FORCE_MS                                       12 /* Force Master/slave Configuration. */
#define BITL_AN_ADV_ABILITY_L_AN_ADV_FORCE_MS                                       1 /* Force Master/slave Configuration. */
#define BITM_AN_ADV_ABILITY_L_AN_ADV_FORCE_MS                                       0x1000 /* Force Master/slave Configuration. */
#define BITP_AN_ADV_ABILITY_L_AN_ADV_REMOTE_FAULT                                   13 /* Remote Fault. */
#define BITL_AN_ADV_ABILITY_L_AN_ADV_REMOTE_FAULT                                   1 /* Remote Fault. */
#define BITM_AN_ADV_ABILITY_L_AN_ADV_REMOTE_FAULT                                   0x2000 /* Remote Fault. */
#define BITP_AN_ADV_ABILITY_L_AN_ADV_ACK                                            14 /* Acknowledge (ACK). */
#define BITL_AN_ADV_ABILITY_L_AN_ADV_ACK                                            1 /* Acknowledge (ACK). */
#define BITM_AN_ADV_ABILITY_L_AN_ADV_ACK                                            0x4000 /* Acknowledge (ACK). */
#define BITP_AN_ADV_ABILITY_L_AN_ADV_NEXT_PAGE_REQ                                  15 /* Next Page Request. */
#define BITL_AN_ADV_ABILITY_L_AN_ADV_NEXT_PAGE_REQ                                  1 /* Next Page Request. */
#define BITM_AN_ADV_ABILITY_L_AN_ADV_NEXT_PAGE_REQ                                  0x8000 /* Next Page Request. */

/* ----------------------------------------------------------------------------------------------------
          AN_ADV_ABILITY_M
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_ADV_ABILITY_M_AN_ADV_MST                                            4 /* Master/slave Configuration. */
#define BITL_AN_ADV_ABILITY_M_AN_ADV_MST                                            1  /* Master/slave Configuration. */
#define BITM_AN_ADV_ABILITY_M_AN_ADV_MST                                            0x0010 /* Master/slave Configuration. */
#define BITP_AN_ADV_ABILITY_M_AN_ADV_B10L                                           14 /* 10BASE-T1L Ability. */
#define BITL_AN_ADV_ABILITY_M_AN_ADV_B10L                                           1 /* 10BASE-T1L Ability. */
#define BITM_AN_ADV_ABILITY_M_AN_ADV_B10L                                           0x4000 /* 10BASE-T1L Ability. */

/* ----------------------------------------------------------------------------------------------------
          AN_ADV_ABILITY_H
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_ADV_ABILITY_H_AN_ADV_B10L_TX_LVL_HI_REQ                             12 /* 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITL_AN_ADV_ABILITY_H_AN_ADV_B10L_TX_LVL_HI_REQ                             1 /* 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITM_AN_ADV_ABILITY_H_AN_ADV_B10L_TX_LVL_HI_REQ                             0x1000 /* 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITP_AN_ADV_ABILITY_H_AN_ADV_B10L_TX_LVL_HI_ABL                             13 /* 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITL_AN_ADV_ABILITY_H_AN_ADV_B10L_TX_LVL_HI_ABL                             1 /* 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITM_AN_ADV_ABILITY_H_AN_ADV_B10L_TX_LVL_HI_ABL                             0x2000 /* 10BASE-T1L High Level Transmit Operating Mode Ability. */

/* ----------------------------------------------------------------------------------------------------
          AN_LP_ADV_ABILITY_L
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_LP_ADV_ABILITY_L_AN_LP_ADV_SELECTOR                                 0 /* Link Partner Selector. */
#define BITL_AN_LP_ADV_ABILITY_L_AN_LP_ADV_SELECTOR                                 5 /* Link Partner Selector. */
#define BITM_AN_LP_ADV_ABILITY_L_AN_LP_ADV_SELECTOR                                 0x001F /* Link Partner Selector. */
#define BITP_AN_LP_ADV_ABILITY_L_AN_LP_ADV_PAUSE                                    10 /* Link Partner Pause Ability. */
#define BITL_AN_LP_ADV_ABILITY_L_AN_LP_ADV_PAUSE                                    2 /* Link Partner Pause Ability. */
#define BITM_AN_LP_ADV_ABILITY_L_AN_LP_ADV_PAUSE                                    0x0C00 /* Link Partner Pause Ability. */
#define BITP_AN_LP_ADV_ABILITY_L_AN_LP_ADV_FORCE_MS                                 12 /* Link Partner Force Master/slave Configuration. */
#define BITL_AN_LP_ADV_ABILITY_L_AN_LP_ADV_FORCE_MS                                 1 /* Link Partner Force Master/slave Configuration. */
#define BITM_AN_LP_ADV_ABILITY_L_AN_LP_ADV_FORCE_MS                                 0x1000 /* Link Partner Force Master/slave Configuration. */
#define BITP_AN_LP_ADV_ABILITY_L_AN_LP_ADV_REMOTE_FAULT                             13 /* Link Partner Remote Fault. */
#define BITL_AN_LP_ADV_ABILITY_L_AN_LP_ADV_REMOTE_FAULT                             1 /* Link Partner Remote Fault. */
#define BITM_AN_LP_ADV_ABILITY_L_AN_LP_ADV_REMOTE_FAULT                             0x2000 /* Link Partner Remote Fault. */
#define BITP_AN_LP_ADV_ABILITY_L_AN_LP_ADV_ACK                                      14 /* Link Partner Acknowledge (ACK). */
#define BITL_AN_LP_ADV_ABILITY_L_AN_LP_ADV_ACK                                      1 /* Link Partner Acknowledge (ACK). */
#define BITM_AN_LP_ADV_ABILITY_L_AN_LP_ADV_ACK                                      0x4000 /* Link Partner Acknowledge (ACK). */
#define BITP_AN_LP_ADV_ABILITY_L_AN_LP_ADV_NEXT_PAGE_REQ                            15 /* Link Partner Next Page Request. */
#define BITL_AN_LP_ADV_ABILITY_L_AN_LP_ADV_NEXT_PAGE_REQ                            1 /* Link Partner Next Page Request. */
#define BITM_AN_LP_ADV_ABILITY_L_AN_LP_ADV_NEXT_PAGE_REQ                            0x8000 /* Link Partner Next Page Request. */

/* ----------------------------------------------------------------------------------------------------
          AN_LP_ADV_ABILITY_M
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_LP_ADV_ABILITY_M_AN_LP_ADV_MST                                      4 /* Link Partner MASTER/_SLAVE Configuration. */
#define BITL_AN_LP_ADV_ABILITY_M_AN_LP_ADV_MST                                      1 /* Link Partner MASTER/_SLAVE Configuration. */
#define BITM_AN_LP_ADV_ABILITY_M_AN_LP_ADV_MST                                      0x0010 /* Link Partner MASTER/_SLAVE Configuration. */
#define BITP_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B100                                     5 /* Link Partner 100BASE-T1 Ability. */
#define BITL_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B100                                     1 /* Link Partner 100BASE-T1 Ability. */
#define BITM_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B100                                     0x0020 /* Link Partner 100BASE-T1 Ability. */
#define BITP_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B10S_FD                                  6 /* Link Partner 10BASE-T1S Full Duplex Ability. */
#define BITL_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B10S_FD                                  1 /* Link Partner 10BASE-T1S Full Duplex Ability. */
#define BITM_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B10S_FD                                  0x0040 /* Link Partner 10BASE-T1S Full Duplex Ability. */
#define BITP_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B1000                                    7 /* Link Partner 1000BASE-T1 Ability. */
#define BITL_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B1000                                    1 /* Link Partner 1000BASE-T1 Ability. */
#define BITM_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B1000                                    0x0080 /* Link Partner 1000BASE-T1 Ability. */
#define BITP_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B10L                                     14 /* Link Partner 10BASE-T1L Ability. */
#define BITL_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B10L                                     1 /* Link Partner 10BASE-T1L Ability. */
#define BITM_AN_LP_ADV_ABILITY_M_AN_LP_ADV_B10L                                     0x4000 /* Link Partner 10BASE-T1L Ability. */

/* ----------------------------------------------------------------------------------------------------
          AN_LP_ADV_ABILITY_H
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10S_HD                                  11 /* Link Partner 10BASE-T1S Half Duplex Ability. */
#define BITL_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10S_HD                                  1 /* Link Partner 10BASE-T1S Half Duplex Ability. */
#define BITM_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10S_HD                                  0x0800 /* Link Partner 10BASE-T1S Half Duplex Ability. */
#define BITP_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10L_TX_LVL_HI_REQ                       12 /* Link Partner 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITL_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10L_TX_LVL_HI_REQ                       1 /* Link Partner 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITM_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10L_TX_LVL_HI_REQ                       0x1000 /* Link Partner 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITP_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10L_TX_LVL_HI_ABL                       13 /* Link Partner 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITL_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10L_TX_LVL_HI_ABL                       1 /* Link Partner 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITM_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10L_TX_LVL_HI_ABL                       0x2000 /* Link Partner 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITP_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10L_EEE                                 14 /* Link Partner 10BASE-T1L EEE Ability. */
#define BITL_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10L_EEE                                 1 /* Link Partner 10BASE-T1L EEE Ability. */
#define BITM_AN_LP_ADV_ABILITY_H_AN_LP_ADV_B10L_EEE                                 0x4000 /* Link Partner 10BASE-T1L EEE Ability. */

/* ----------------------------------------------------------------------------------------------------
          AN_NEXT_PAGE_L
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_NEXT_PAGE_L_AN_NP_MESSAGE_CODE                                      0 /* Message/unformatted Code Field. */
#define BITL_AN_NEXT_PAGE_L_AN_NP_MESSAGE_CODE                                      11 /* Message/unformatted Code Field. */
#define BITM_AN_NEXT_PAGE_L_AN_NP_MESSAGE_CODE                                      0x07FF /* Message/unformatted Code Field. */
#define BITP_AN_NEXT_PAGE_L_AN_NP_TOGGLE                                            11 /* Toggle Bit. */
#define BITL_AN_NEXT_PAGE_L_AN_NP_TOGGLE                                            1 /* Toggle Bit. */
#define BITM_AN_NEXT_PAGE_L_AN_NP_TOGGLE                                            0x0800 /* Toggle Bit. */
#define BITP_AN_NEXT_PAGE_L_AN_NP_ACK2                                              12 /* Acknowledge 2. */
#define BITL_AN_NEXT_PAGE_L_AN_NP_ACK2                                              1 /* Acknowledge 2. */
#define BITM_AN_NEXT_PAGE_L_AN_NP_ACK2                                              0x1000 /* Acknowledge 2. */
#define BITP_AN_NEXT_PAGE_L_AN_NP_MESSAGE_PAGE                                      13 /* Next Page Encoding. */
#define BITL_AN_NEXT_PAGE_L_AN_NP_MESSAGE_PAGE                                      1 /* Next Page Encoding. */
#define BITM_AN_NEXT_PAGE_L_AN_NP_MESSAGE_PAGE                                      0x2000 /* Next Page Encoding. */
#define BITP_AN_NEXT_PAGE_L_AN_NP_ACK                                               14 /* Next Page Acknowledge. */
#define BITL_AN_NEXT_PAGE_L_AN_NP_ACK                                               1 /* Next Page Acknowledge. */
#define BITM_AN_NEXT_PAGE_L_AN_NP_ACK                                               0x4000 /* Next Page Acknowledge. */
#define BITP_AN_NEXT_PAGE_L_AN_NP_NEXT_PAGE_REQ                                     15 /* Next Page Request. */
#define BITL_AN_NEXT_PAGE_L_AN_NP_NEXT_PAGE_REQ                                     1 /* Next Page Request. */
#define BITM_AN_NEXT_PAGE_L_AN_NP_NEXT_PAGE_REQ                                     0x8000 /* Next Page Request. */

#define ENUM_AN_NEXT_PAGE_L_AN_NP_MESSAGE_PAGE_NP_UNFORMATTED_NEXT_PAGE             0x0000 /* Unformatted next page */
#define ENUM_AN_NEXT_PAGE_L_AN_NP_MESSAGE_PAGE_NP_MESSAGE_NEXT_PAGE                 0x0001 /* Message next page */
#define ENUM_AN_NEXT_PAGE_L_AN_NP_MESSAGE_CODE_NP_MESSAGE_NULL                      0x0001 /* Null Message */
#define ENUM_AN_NEXT_PAGE_L_AN_NP_MESSAGE_CODE_NP_MESSAGE_OUI                       0x0005 /* Organizationally Unique Identifier Tagged Message */
#define ENUM_AN_NEXT_PAGE_L_AN_NP_MESSAGE_CODE_NP_MESSAGE_AN_DEV_ID                 0x0006 /* AN Device Identifier Tag Code */

/* ----------------------------------------------------------------------------------------------------
          AN_NEXT_PAGE_M
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_NEXT_PAGE_M_AN_NP_UNFORMATTED1                                      0 /* Unformatted Code Field 1 */
#define BITL_AN_NEXT_PAGE_M_AN_NP_UNFORMATTED1                                      16 /* Unformatted Code Field 1 */
#define BITM_AN_NEXT_PAGE_M_AN_NP_UNFORMATTED1                                      0xFFFF /* Unformatted Code Field 1 */

/* ----------------------------------------------------------------------------------------------------
          AN_NEXT_PAGE_H
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_NEXT_PAGE_H_AN_NP_UNFORMATTED2                                      0 /* Unformatted Code Field 2 */
#define BITL_AN_NEXT_PAGE_H_AN_NP_UNFORMATTED2                                      16 /* Unformatted Code Field 2 */
#define BITM_AN_NEXT_PAGE_H_AN_NP_UNFORMATTED2                                      0xFFFF /* Unformatted Code Field 2 */

/* ----------------------------------------------------------------------------------------------------
          AN_LP_NEXT_PAGE_L
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_CODE                                0 /* Link Partner Message/unformatted Code Field. */
#define BITL_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_CODE                                11 /* Link Partner Message/unformatted Code Field. */
#define BITM_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_CODE                                0x07FF /* Link Partner Message/unformatted Code Field. */
#define BITP_AN_LP_NEXT_PAGE_L_AN_LP_NP_TOGGLE                                      11 /* Link Partner Toggle Bit. */
#define BITL_AN_LP_NEXT_PAGE_L_AN_LP_NP_TOGGLE                                      1 /* Link Partner Toggle Bit. */
#define BITM_AN_LP_NEXT_PAGE_L_AN_LP_NP_TOGGLE                                      0x0800 /* Link Partner Toggle Bit. */
#define BITP_AN_LP_NEXT_PAGE_L_AN_LP_NP_ACK2                                        12 /* Link Partner Acknowledge 2. */
#define BITL_AN_LP_NEXT_PAGE_L_AN_LP_NP_ACK2                                        1 /* Link Partner Acknowledge 2. */
#define BITM_AN_LP_NEXT_PAGE_L_AN_LP_NP_ACK2                                        0x1000 /* Link Partner Acknowledge 2. */
#define BITP_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_PAGE                                13 /* Link Partner Next Page Encoding. */
#define BITL_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_PAGE                                1 /* Link Partner Next Page Encoding. */
#define BITM_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_PAGE                                0x2000 /* Link Partner Next Page Encoding. */
#define BITP_AN_LP_NEXT_PAGE_L_AN_LP_NP_ACK                                         14 /* Link Partner Next Page Acknowledge. */
#define BITL_AN_LP_NEXT_PAGE_L_AN_LP_NP_ACK                                         1 /* Link Partner Next Page Acknowledge. */
#define BITM_AN_LP_NEXT_PAGE_L_AN_LP_NP_ACK                                         0x4000 /* Link Partner Next Page Acknowledge. */
#define BITP_AN_LP_NEXT_PAGE_L_AN_LP_NP_NEXT_PAGE_REQ                               15 /* Next Page Request. */
#define BITL_AN_LP_NEXT_PAGE_L_AN_LP_NP_NEXT_PAGE_REQ                               1 /* Next Page Request. */
#define BITM_AN_LP_NEXT_PAGE_L_AN_LP_NP_NEXT_PAGE_REQ                               0x8000 /* Next Page Request. */

#define ENUM_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_PAGE_NP_UNFORMATTED_NEXT_PAGE       0x0000 /* Unformatted next page */
#define ENUM_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_PAGE_NP_MESSAGE_NEXT_PAGE           0x0001 /* Message next page */
#define ENUM_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_CODE_NP_MESSAGE_NULL                0x0001 /* Null Message */
#define ENUM_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_CODE_NP_MESSAGE_OUI                 0x0005 /* Organizationally Unique Identifier Tagged Message */
#define ENUM_AN_LP_NEXT_PAGE_L_AN_LP_NP_MESSAGE_CODE_NP_MESSAGE_AN_DEV_ID           0x0006 /* AN Device Identifier Tag Code */

/* ----------------------------------------------------------------------------------------------------
          AN_LP_NEXT_PAGE_M
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_LP_NEXT_PAGE_M_AN_LP_NP_UNFORMATTED1                                0 /* Link Partner Unformatted Code Field 1 */
#define BITL_AN_LP_NEXT_PAGE_M_AN_LP_NP_UNFORMATTED1                                16 /* Link Partner Unformatted Code Field 1 */
#define BITM_AN_LP_NEXT_PAGE_M_AN_LP_NP_UNFORMATTED1                                0xFFFF /* Link Partner Unformatted Code Field 1 */

/* ----------------------------------------------------------------------------------------------------
          AN_LP_NEXT_PAGE_H
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_LP_NEXT_PAGE_H_AN_LP_NP_UNFORMATTED2                                0 /* Link Partner Unformatted Code Field 2 */
#define BITL_AN_LP_NEXT_PAGE_H_AN_LP_NP_UNFORMATTED2                                16 /* Link Partner Unformatted Code Field 2 */
#define BITM_AN_LP_NEXT_PAGE_H_AN_LP_NP_UNFORMATTED2                                0xFFFF /* Link Partner Unformatted Code Field 2 */

/* ----------------------------------------------------------------------------------------------------
          AN_B10_ADV_ABILITY
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L_TX_LVL_HI_REQ                       12 /* 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITL_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L_TX_LVL_HI_REQ                       1 /* 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITM_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L_TX_LVL_HI_REQ                       0x1000 /* 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITP_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L_TX_LVL_HI_ABL                       13 /* 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITL_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L_TX_LVL_HI_ABL                       1 /* 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITM_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L_TX_LVL_HI_ABL                       0x2000 /* 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITP_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L_EEE                                 14 /* 10BASE-T1L EEE Ability. */
#define BITL_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L_EEE                                 1 /* 10BASE-T1L EEE Ability. */
#define BITM_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L_EEE                                 0x4000 /* 10BASE-T1L EEE Ability. */
#define BITP_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L                                     15 /* 10BASE-T1L Ability. */
#define BITL_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L                                     1 /* 10BASE-T1L Ability. */
#define BITM_AN_B10_ADV_ABILITY_AN_B10_ADV_B10L                                     0x8000 /* 10BASE-T1L Ability. */

/* ----------------------------------------------------------------------------------------------------
          AN_B10_LP_ADV_ABILITY
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10S_HD                            6 /* Link Partner 10BASE-T1S Half Duplex Ability. */
#define BITL_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10S_HD                            1 /* Link Partner 10BASE-T1S Half Duplex Ability. */
#define BITM_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10S_HD                            0x0040 /* Link Partner 10BASE-T1S Half Duplex Ability. */
#define BITP_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10S_FD                            7 /* Link Partner 10BASE-T1S Full Duplex Ability. */
#define BITL_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10S_FD                            1 /* Link Partner 10BASE-T1S Full Duplex Ability. */
#define BITM_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10S_FD                            0x0080 /* Link Partner 10BASE-T1S Full Duplex Ability. */
#define BITP_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L_TX_LVL_HI_REQ                 12 /* 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITL_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L_TX_LVL_HI_REQ                 1 /* 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITM_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L_TX_LVL_HI_REQ                 0x1000 /* 10BASE-T1L High Level Transmit Operating Mode Request. */
#define BITP_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L_TX_LVL_HI_ABL                 13/* 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITL_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L_TX_LVL_HI_ABL                 1 /* 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITM_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L_TX_LVL_HI_ABL                 0x2000 /* 10BASE-T1L High Level Transmit Operating Mode Ability. */
#define BITP_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L_EEE                           14 /* 10BASE-T1L EEE Ability. */
#define BITL_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L_EEE                           1 /* 10BASE-T1L EEE Ability. */
#define BITM_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L_EEE                           0x4000 /* 10BASE-T1L EEE Ability. */
#define BITP_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L                               15 /* 10BASE-T1L Ability. */
#define BITL_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L                               1 /* 10BASE-T1L Ability. */
#define BITM_AN_B10_LP_ADV_ABILITY_AN_B10_LP_ADV_B10L                               0x8000 /* 10BASE-T1L Ability. */

/* ----------------------------------------------------------------------------------------------------
          AN_FRC_MODE_EN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_FRC_MODE_EN_AN_FRC_MODE_EN                                          0 /* Autonegotiation Forced Mode. */
#define BITL_AN_FRC_MODE_EN_AN_FRC_MODE_EN                                          1 /* Autonegotiation Forced Mode. */
#define BITM_AN_FRC_MODE_EN_AN_FRC_MODE_EN                                          0x0001 /* Autonegotiation Forced Mode. */

/* ----------------------------------------------------------------------------------------------------
          AN_STATUS_EXTRA
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_STATUS_EXTRA_AN_LINK_GOOD                                           0 /* Autonegotiation Complete Indication. */
#define BITL_AN_STATUS_EXTRA_AN_LINK_GOOD                                           1 /* Autonegotiation Complete Indication. */
#define BITM_AN_STATUS_EXTRA_AN_LINK_GOOD                                           0x0001 /* Autonegotiation Complete Indication. */
#define BITP_AN_STATUS_EXTRA_AN_HCD_TECH                                            1 /* Highest Common Denominator (HCD) PHY Technology. */
#define BITL_AN_STATUS_EXTRA_AN_HCD_TECH                                            4 /* Highest Common Denominator (HCD) PHY Technology. */
#define BITM_AN_STATUS_EXTRA_AN_HCD_TECH                                            0x001E /* Highest Common Denominator (HCD) PHY Technology. */
#define BITP_AN_STATUS_EXTRA_AN_MS_CONFIG_RSLTN                                     5 /* Master/slave Resolution Result. */
#define BITL_AN_STATUS_EXTRA_AN_MS_CONFIG_RSLTN                                     2 /* Master/slave Resolution Result. */
#define BITM_AN_STATUS_EXTRA_AN_MS_CONFIG_RSLTN                                     0x0060 /* Master/slave Resolution Result. */
#define BITP_AN_STATUS_EXTRA_AN_TX_LVL_RSLTN                                        7 /* Autonegotiation Tx Level Result */
#define BITL_AN_STATUS_EXTRA_AN_TX_LVL_RSLTN                                        2 /* Autonegotiation Tx Level Result */
#define BITM_AN_STATUS_EXTRA_AN_TX_LVL_RSLTN                                        0x0180 /* Autonegotiation Tx Level Result */
#define BITP_AN_STATUS_EXTRA_AN_INC_LINK                                            9 /* Incompatible Link Indication. */
#define BITL_AN_STATUS_EXTRA_AN_INC_LINK                                            1 /* Incompatible Link Indication. */
#define BITM_AN_STATUS_EXTRA_AN_INC_LINK                                            0x0200 /* Incompatible Link Indication. */
#define BITP_AN_STATUS_EXTRA_AN_LP_NP_RX                                            10 /* Next Page Request Received from Link Partner. */
#define BITL_AN_STATUS_EXTRA_AN_LP_NP_RX                                            1 /* Next Page Request Received from Link Partner. */
#define BITM_AN_STATUS_EXTRA_AN_LP_NP_RX                                            0x0400 /* Next Page Request Received from Link Partner. */

#define ENUM_AN_STATUS_EXTRA_AN_TX_LVL_RSLTN_TX_LVL_NOT_RUN                         0x0000 /* Not run */
#define ENUM_AN_STATUS_EXTRA_AN_TX_LVL_RSLTN_TX_LVL_LOW                             0x0002 /* Success, low transmit levels (1.0 Vpp) selected */
#define ENUM_AN_STATUS_EXTRA_AN_TX_LVL_RSLTN_TX_LVL_HIGH                            0x0003 /* Success, high transmit levels (2.4 Vpp) selected */
#define ENUM_AN_STATUS_EXTRA_AN_MS_CONFIG_RSLTN_MS_NOT_RUN                          0x0000 /* Not run */
#define ENUM_AN_STATUS_EXTRA_AN_MS_CONFIG_RSLTN_MS_FAULT                            0x0001 /* Configuration fault */
#define ENUM_AN_STATUS_EXTRA_AN_MS_CONFIG_RSLTN_MS_SLAVE                            0x0002 /* Success, PHY is configured as SLAVE */
#define ENUM_AN_STATUS_EXTRA_AN_MS_CONFIG_RSLTN_MS_MASTER                           0x0003 /* Success, PHY is configured as MASTER */
#define ENUM_AN_STATUS_EXTRA_AN_HCD_TECH_HCD_NULL                                   0x0000 /* NULL (not run) */
#define ENUM_AN_STATUS_EXTRA_AN_HCD_TECH_HCD_B10L                                   0x0001 /* 10BASE-T1L */

/* ----------------------------------------------------------------------------------------------------
          AN_PHY_INST_STATUS
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AN_PHY_INST_STATUS_IS_TX_LVL_LO                                        0 /* Tx Level Low Status */
#define BITL_AN_PHY_INST_STATUS_IS_TX_LVL_LO                                        1 /* Tx Level Low Status */
#define BITM_AN_PHY_INST_STATUS_IS_TX_LVL_LO                                        0x0001 /* Tx Level Low Status */
#define BITP_AN_PHY_INST_STATUS_IS_TX_LVL_HI                                        1 /* Tx Level High Status */
#define BITL_AN_PHY_INST_STATUS_IS_TX_LVL_HI                                        1 /* Tx Level High Status */
#define BITM_AN_PHY_INST_STATUS_IS_TX_LVL_HI                                        0x0002 /* Tx Level High Status */
#define BITP_AN_PHY_INST_STATUS_IS_CFG_SLV                                          2 /* Slave Status */
#define BITL_AN_PHY_INST_STATUS_IS_CFG_SLV                                          1 /* Slave Status */
#define BITM_AN_PHY_INST_STATUS_IS_CFG_SLV                                          0x0004 /* Slave Status */
#define BITP_AN_PHY_INST_STATUS_IS_CFG_MST                                          3 /* Master Status */
#define BITL_AN_PHY_INST_STATUS_IS_CFG_MST                                          1 /* Master Status */
#define BITM_AN_PHY_INST_STATUS_IS_CFG_MST                                          0x0008 /* Master Status */
#define BITP_AN_PHY_INST_STATUS_IS_AN_TX_EN                                         4 /* Autonegotiation Tx Enable Status */
#define BITL_AN_PHY_INST_STATUS_IS_AN_TX_EN                                         1 /* Autonegotiation Tx Enable Status */
#define BITM_AN_PHY_INST_STATUS_IS_AN_TX_EN                                         0x0010 /* Autonegotiation Tx Enable Status */

/* ----------------------------------------------------------------------------------------------------
          MMD1_DEV_ID1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD1_DEV_ID1_MMD1_DEV_ID1                                              0 /* Organizationally Unique Identifier */
#define BITL_MMD1_DEV_ID1_MMD1_DEV_ID1                                              16 /* Organizationally Unique Identifier */
#define BITM_MMD1_DEV_ID1_MMD1_DEV_ID1                                              0xFFFF/* Organizationally Unique Identifier */

/* ----------------------------------------------------------------------------------------------------
          MMD1_DEV_ID2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD1_DEV_ID2_MMD1_REV_NUM                                              0 /* Revision Number */
#define BITL_MMD1_DEV_ID2_MMD1_REV_NUM                                              4 /* Revision Number */
#define BITM_MMD1_DEV_ID2_MMD1_REV_NUM                                              0x000F /* Revision Number */
#define BITP_MMD1_DEV_ID2_MMD1_MODEL_NUM                                            4 /* Model Number */
#define BITL_MMD1_DEV_ID2_MMD1_MODEL_NUM                                            6 /* Model Number */
#define BITM_MMD1_DEV_ID2_MMD1_MODEL_NUM                                            0x03F0 /* Model Number */
#define BITP_MMD1_DEV_ID2_MMD1_DEV_ID2_OUI                                          10 /* Organizationally Unique Identifier */
#define BITL_MMD1_DEV_ID2_MMD1_DEV_ID2_OUI                                          6 /* Organizationally Unique Identifier */
#define BITM_MMD1_DEV_ID2_MMD1_DEV_ID2_OUI                                          0xFC00 /* Organizationally Unique Identifier */

/* ----------------------------------------------------------------------------------------------------
          MMD1_DEVS_IN_PKG1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD1_DEVS_IN_PKG1_MMD1_DEVS_IN_PKG1                                    0 /* Vendor specific 1 MMD Devices in Package */
#define BITL_MMD1_DEVS_IN_PKG1_MMD1_DEVS_IN_PKG1                                    16 /* Vendor specific 1 MMD Devices in Package */
#define BITM_MMD1_DEVS_IN_PKG1_MMD1_DEVS_IN_PKG1                                    0xFFFF /* Vendor specific 1 MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          MMD1_DEVS_IN_PKG2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD1_DEVS_IN_PKG2_MMD1_DEVS_IN_PKG2                                    0 /* Vendor specific 1 MMD Devices in Package */
#define BITL_MMD1_DEVS_IN_PKG2_MMD1_DEVS_IN_PKG2                                    16 /* Vendor specific 1 MMD Devices in Package */
#define BITM_MMD1_DEVS_IN_PKG2_MMD1_DEVS_IN_PKG2                                    0xFFFF /* Vendor specific 1 MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          MMD1_STATUS
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD1_STATUS_MMD1_STATUS                                                14 /* Vendor Specific 1 MMD Status */
#define BITL_MMD1_STATUS_MMD1_STATUS                                                2 /* Vendor Specific 1 MMD Status */
#define BITM_MMD1_STATUS_MMD1_STATUS                                                0xC000 /* Vendor Specific 1 MMD Status */

/* ----------------------------------------------------------------------------------------------------
          CRSM_IRQ_STATUS
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CRSM_IRQ_STATUS_CRSM_HRD_RST_IRQ_LH                                    12 /* Hardware Reset Interrupt */
#define BITL_CRSM_IRQ_STATUS_CRSM_HRD_RST_IRQ_LH                                    1 /* Hardware Reset Interrupt */
#define BITM_CRSM_IRQ_STATUS_CRSM_HRD_RST_IRQ_LH                                    0x1000 /* Hardware Reset Interrupt */
#define BITP_CRSM_IRQ_STATUS_CRSM_SW_IRQ_LH                                         15 /* Software Requested Interrupt Event. */
#define BITL_CRSM_IRQ_STATUS_CRSM_SW_IRQ_LH                                         1 /* Software Requested Interrupt Event. */
#define BITM_CRSM_IRQ_STATUS_CRSM_SW_IRQ_LH                                         0x8000 /* Software Requested Interrupt Event. */

/* ----------------------------------------------------------------------------------------------------
          CRSM_IRQ_MASK
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CRSM_IRQ_MASK_CRSM_HRD_RST_IRQ_EN                                      12 /* Enable Hardware Reset Interrupt. */
#define BITL_CRSM_IRQ_MASK_CRSM_HRD_RST_IRQ_EN                                      1 /* Enable Hardware Reset Interrupt. */
#define BITM_CRSM_IRQ_MASK_CRSM_HRD_RST_IRQ_EN                                      0x1000 /* Enable Hardware Reset Interrupt. */
#define BITP_CRSM_IRQ_MASK_CRSM_SW_IRQ_REQ                                          15 /* Software Interrupt Request. */
#define BITL_CRSM_IRQ_MASK_CRSM_SW_IRQ_REQ                                          1 /* Software Interrupt Request. */
#define BITM_CRSM_IRQ_MASK_CRSM_SW_IRQ_REQ                                          0x8000 /* Software Interrupt Request. */

/* ----------------------------------------------------------------------------------------------------
          CRSM_SFT_RST
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CRSM_SFT_RST_CRSM_SFT_RST                                              0 /* Software Reset Register. */
#define BITL_CRSM_SFT_RST_CRSM_SFT_RST                                              1 /* Software Reset Register. */
#define BITM_CRSM_SFT_RST_CRSM_SFT_RST                                              0x0001 /* Software Reset Register. */

/* ----------------------------------------------------------------------------------------------------
          CRSM_SFT_PD_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CRSM_SFT_PD_CNTRL_CRSM_SFT_PD                                          0 /* Software Power-down */
#define BITL_CRSM_SFT_PD_CNTRL_CRSM_SFT_PD                                          1 /* Software Power-down */
#define BITM_CRSM_SFT_PD_CNTRL_CRSM_SFT_PD                                          0x0001 /* Software Power-down */

/* ----------------------------------------------------------------------------------------------------
          CRSM_PHY_SUBSYS_RST
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CRSM_PHY_SUBSYS_RST_CRSM_PHY_SUBSYS_RST                                0 /* PHY Subsystem Reset */
#define BITL_CRSM_PHY_SUBSYS_RST_CRSM_PHY_SUBSYS_RST                                1 /* PHY Subsystem Reset */
#define BITM_CRSM_PHY_SUBSYS_RST_CRSM_PHY_SUBSYS_RST                                0x0001 /* PHY Subsystem Reset */

/* ----------------------------------------------------------------------------------------------------
          CRSM_MAC_IF_RST
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CRSM_MAC_IF_RST_CRSM_MAC_IF_RST                                        0 /* PHY MAC Interface Reset */
#define BITL_CRSM_MAC_IF_RST_CRSM_MAC_IF_RST                                        1 /* PHY MAC Interface Reset */
#define BITM_CRSM_MAC_IF_RST_CRSM_MAC_IF_RST                                        0x0001 /* PHY MAC Interface Reset */

/* ----------------------------------------------------------------------------------------------------
          CRSM_STAT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CRSM_STAT_CRSM_SYS_RDY                                                 0 /* System Ready */
#define BITL_CRSM_STAT_CRSM_SYS_RDY                                                 1 /* System Ready */
#define BITM_CRSM_STAT_CRSM_SYS_RDY                                                 0x0001 /* System Ready */
#define BITP_CRSM_STAT_CRSM_SFT_PD_RDY                                              1 /* Software Power-down Status */
#define BITL_CRSM_STAT_CRSM_SFT_PD_RDY                                              1 /* Software Power-down Status */
#define BITM_CRSM_STAT_CRSM_SFT_PD_RDY                                              0x0002 /* Software Power-down Status */

/* ----------------------------------------------------------------------------------------------------
          CRSM_PMG_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CRSM_PMG_CNTRL_CRSM_FRC_OSC_EN                                         0 /* Force Digital Boot Oscillator Clock Enable */
#define BITL_CRSM_PMG_CNTRL_CRSM_FRC_OSC_EN                                         1 /* Force Digital Boot Oscillator Clock Enable */
#define BITM_CRSM_PMG_CNTRL_CRSM_FRC_OSC_EN                                         0x0001 /* Force Digital Boot Oscillator Clock Enable */

/* ----------------------------------------------------------------------------------------------------
          CRSM_DIAG_CLK_CTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CRSM_DIAG_CLK_CTRL_CRSM_DIAG_CLK_EN                                    0 /* Enable the Diagnostics Clock */
#define BITL_CRSM_DIAG_CLK_CTRL_CRSM_DIAG_CLK_EN                                    1 /* Enable the Diagnostics Clock */
#define BITM_CRSM_DIAG_CLK_CTRL_CRSM_DIAG_CLK_EN                                    0x0001 /* Enable the Diagnostics Clock */

/* ----------------------------------------------------------------------------------------------------
          MGMT_PRT_PKG
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MGMT_PRT_PKG_MGMT_PRT_PKG_VAL                                          0 /* Package Type */
#define BITL_MGMT_PRT_PKG_MGMT_PRT_PKG_VAL                                          6 /* Package Type */
#define BITM_MGMT_PRT_PKG_MGMT_PRT_PKG_VAL                                          0x003F /* Package Type */

/* ----------------------------------------------------------------------------------------------------
          MGMT_MDIO_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MGMT_MDIO_CNTRL_MGMT_GRP_MDIO_EN                                       0 /* Enable MDIO PHY/_PORT Group Address Mode */
#define BITL_MGMT_MDIO_CNTRL_MGMT_GRP_MDIO_EN                                       1 /* Enable MDIO PHY/_PORT Group Address Mode */
#define BITM_MGMT_MDIO_CNTRL_MGMT_GRP_MDIO_EN                                       0x0001 /* Enable MDIO PHY/_PORT Group Address Mode */

/* ----------------------------------------------------------------------------------------------------
          DIGIO_PINMUX
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DIGIO_PINMUX_DIGIO_LINK_ST_POLARITY                                    0 /* LINK_ST Polarity */
#define BITL_DIGIO_PINMUX_DIGIO_LINK_ST_POLARITY                                    1 /* LINK_ST Polarity */
#define BITM_DIGIO_PINMUX_DIGIO_LINK_ST_POLARITY                                    0x0001 /* LINK_ST Polarity */
#define BITP_DIGIO_PINMUX_DIGIO_LED1_PINMUX                                         1 /* Pin Mux Select for LED_1 */
#define BITL_DIGIO_PINMUX_DIGIO_LED1_PINMUX                                         3 /* Pin Mux Select for LED_1 */
#define BITM_DIGIO_PINMUX_DIGIO_LED1_PINMUX                                         0x000E /* Pin Mux Select for LED_1 */
#define BITP_DIGIO_PINMUX_DIGIO_TSCAPT_PINMUX                                       4 /* Pin Mux Select for TS_CAPT */
#define BITL_DIGIO_PINMUX_DIGIO_TSCAPT_PINMUX                                       2 /* Pin Mux Select for TS_CAPT */
#define BITM_DIGIO_PINMUX_DIGIO_TSCAPT_PINMUX                                       0x0030 /* Pin Mux Select for TS_CAPT */
#define BITP_DIGIO_PINMUX_DIGIO_TSTIMER_PINMUX                                      6 /* Pin Mux Select for TS_TIMER */
#define BITL_DIGIO_PINMUX_DIGIO_TSTIMER_PINMUX                                      2 /* Pin Mux Select for TS_TIMER */
#define BITM_DIGIO_PINMUX_DIGIO_TSTIMER_PINMUX                                      0x00C0 /* Pin Mux Select for TS_TIMER */

#define ENUM_DIGIO_PINMUX_DIGIO_TSTIMER_PINMUX_RXD_1                                0x0000 /* RXD_1 */
#define ENUM_DIGIO_PINMUX_DIGIO_TSTIMER_PINMUX_LED_0                                0x0001 /* LED_0 */
#define ENUM_DIGIO_PINMUX_DIGIO_TSTIMER_PINMUX_INT_N                                0x0002 /* INT_N */
#define ENUM_DIGIO_PINMUX_DIGIO_TSTIMER_PINMUX_TS_TIMER_NA                          0x0003 /* TS_TIMER not assigned */
#define ENUM_DIGIO_PINMUX_DIGIO_LED1_PINMUX_LED_1                                   0x0000 /* LED_1 */
#define ENUM_DIGIO_PINMUX_DIGIO_LED1_PINMUX_TX_ER                                   0x0001 /* TX_ER */
#define ENUM_DIGIO_PINMUX_DIGIO_LED1_PINMUX_TX_EN                                   0x0002 /* TX_EN */
#define ENUM_DIGIO_PINMUX_DIGIO_LED1_PINMUX_TX_CLK                                  0x0003 /* TX_CLK */
#define ENUM_DIGIO_PINMUX_DIGIO_LED1_PINMUX_TXD_0                                   0x0004 /* TXD_0 */
#define ENUM_DIGIO_PINMUX_DIGIO_LED1_PINMUX_TXD_2                                   0x0005 /* TXD_2 */
#define ENUM_DIGIO_PINMUX_DIGIO_LED1_PINMUX_LINK_ST                                 0x0006 /* LINK_ST */
#define ENUM_DIGIO_PINMUX_DIGIO_LED1_PINMUX_LED_1_NA                                0x0007 /* LED_1 output not enabled */
#define ENUM_DIGIO_PINMUX_DIGIO_LINK_ST_POLARITY_ASSERT_HIGH                        0x0000 /* ASSERT_HIGH */
#define ENUM_DIGIO_PINMUX_DIGIO_LINK_ST_POLARITY_ASSERT_LOW                         0x0001 /* ASSERT_LOW */

/* ----------------------------------------------------------------------------------------------------
          LED0_BLINK_TIME_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LED0_BLINK_TIME_CNTRL_LED0_OFF_N4MS                                    0 /* Led 0 Off Blink Time. */
#define BITL_LED0_BLINK_TIME_CNTRL_LED0_OFF_N4MS                                    8 /* Led 0 Off Blink Time. */
#define BITM_LED0_BLINK_TIME_CNTRL_LED0_OFF_N4MS                                    0x00FF /* Led 0 Off Blink Time. */
#define BITP_LED0_BLINK_TIME_CNTRL_LED0_ON_N4MS                                     8 /* LED 0 On Blink Time */
#define BITL_LED0_BLINK_TIME_CNTRL_LED0_ON_N4MS                                     8 /* LED 0 On Blink Time */
#define BITM_LED0_BLINK_TIME_CNTRL_LED0_ON_N4MS                                     0xFF00 /* LED 0 On Blink Time */

/* ----------------------------------------------------------------------------------------------------
          LED1_BLINK_TIME_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LED1_BLINK_TIME_CNTRL_LED1_OFF_N4MS                                    0 /* LED 1 Off Blink Time */
#define BITL_LED1_BLINK_TIME_CNTRL_LED1_OFF_N4MS                                    8 /* LED 1 Off Blink Time */
#define BITM_LED1_BLINK_TIME_CNTRL_LED1_OFF_N4MS                                    0x00FF /* LED 1 Off Blink Time */
#define BITP_LED1_BLINK_TIME_CNTRL_LED1_ON_N4MS                                     8 /* LED 1 On Blink Time */
#define BITL_LED1_BLINK_TIME_CNTRL_LED1_ON_N4MS                                     8 /* LED 1 On Blink Time */
#define BITM_LED1_BLINK_TIME_CNTRL_LED1_ON_N4MS                                     0xFF00 /* LED 1 On Blink Time */

/* ----------------------------------------------------------------------------------------------------
          LED_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LED_CNTRL_LED0_FUNCTION                                                0 /* LED 0 Pin Function */
#define BITL_LED_CNTRL_LED0_FUNCTION                                                5 /* LED 0 Pin Function */
#define BITM_LED_CNTRL_LED0_FUNCTION                                                0x001F /* LED 0 Pin Function */
#define BITP_LED_CNTRL_LED0_MODE                                                    5 /* LED 0 Mode Selection */
#define BITL_LED_CNTRL_LED0_MODE                                                    1 /* LED 0 Mode Selection */
#define BITM_LED_CNTRL_LED0_MODE                                                    0x0020 /* LED 0 Mode Selection */
#define BITP_LED_CNTRL_LED0_LINK_ST_QUALIFY                                         6 /* Qualify Certain LED 0 Options with LINK_STATUS */
#define BITL_LED_CNTRL_LED0_LINK_ST_QUALIFY                                         1 /* Qualify Certain LED 0 Options with LINK_STATUS */
#define BITM_LED_CNTRL_LED0_LINK_ST_QUALIFY                                         0x0040 /* Qualify Certain LED 0 Options with LINK_STATUS */
#define BITP_LED_CNTRL_LED0_EN                                                      7 /* LED 0 Enable */
#define BITL_LED_CNTRL_LED0_EN                                                      1 /* LED 0 Enable */
#define BITM_LED_CNTRL_LED0_EN                                                      0x0080 /* LED 0 Enable */
#define BITP_LED_CNTRL_LED1_FUNCTION                                                8 /* LED 1 Pin Function */
#define BITL_LED_CNTRL_LED1_FUNCTION                                                5 /* LED 1 Pin Function */
#define BITM_LED_CNTRL_LED1_FUNCTION                                                0x1F00 /* LED 1 Pin Function */
#define BITP_LED_CNTRL_LED1_MODE                                                    13 /* LED 1 Mode Selection */
#define BITL_LED_CNTRL_LED1_MODE                                                    1 /* LED 1 Mode Selection */
#define BITM_LED_CNTRL_LED1_MODE                                                    0x2000 /* LED 1 Mode Selection */
#define BITP_LED_CNTRL_LED1_LINK_ST_QUALIFY                                         14 /* Qualify Certain LED 1 Options with LINK_STATUS */
#define BITL_LED_CNTRL_LED1_LINK_ST_QUALIFY                                         1 /* Qualify Certain LED 1 Options with LINK_STATUS */
#define BITM_LED_CNTRL_LED1_LINK_ST_QUALIFY                                         0x4000U /* Qualify Certain LED 1 Options with LINK_STATUS */
#define BITP_LED_CNTRL_LED1_EN                                                      15 /* LED 1 Enable */
#define BITL_LED_CNTRL_LED1_EN                                                      1 /* LED 1 Enable */
#define BITM_LED_CNTRL_LED1_EN                                                      0x8000 /* LED 1 Enable */

#define ENUM_LED_CNTRL_LED1_MODE_LED_MODE1                                          0x0000 /* LED_MODE1 */
#define ENUM_LED_CNTRL_LED1_MODE_LED_MODE2                                          0x0001 /* LED_MODE2 */
#define ENUM_LED_CNTRL_LED1_FUNCTION_LINKUP_TXRX_ACTIVITY                           0x0000 /* LINKUP_TXRX_ACTIVITY */
#define ENUM_LED_CNTRL_LED1_FUNCTION_LINKUP_TX_ACTIVITY                             0x0001 /* LINKUP_TX_ACTIVITY */
#define ENUM_LED_CNTRL_LED1_FUNCTION_LINKUP_RX_ACTIVITY                             0x0002 /* LINKUP_RX_ACTIVITY */
#define ENUM_LED_CNTRL_LED1_FUNCTION_LINKUP_ONLY                                    0x0003 /* LINKUP_ONLY */
#define ENUM_LED_CNTRL_LED1_FUNCTION_TXRX_ACTIVITY                                  0x0004 /* TXRX_ACTIVITY */
#define ENUM_LED_CNTRL_LED1_FUNCTION_TX_ACTIVITY                                    0x0005 /* TX_ACTIVITY */
#define ENUM_LED_CNTRL_LED1_FUNCTION_RX_ACTIVITY                                    0x0006 /* RX_ACTIVITY */
#define ENUM_LED_CNTRL_LED1_FUNCTION_LINKUP_RX_ER                                   0x0007 /* LINKUP_RX_ER */
#define ENUM_LED_CNTRL_LED1_FUNCTION_LINKUP_RX_TX_ER                                0x0008 /* LINKUP_RX_TX_ER */
#define ENUM_LED_CNTRL_LED1_FUNCTION_RX_ER                                          0x0009 /* RX_ER */
#define ENUM_LED_CNTRL_LED1_FUNCTION_RX_TX_ER                                       0x000A /* RX_TX_ER */
#define ENUM_LED_CNTRL_LED1_FUNCTION_TX_SOP                                         0x000B /* TX_SOP */
#define ENUM_LED_CNTRL_LED1_FUNCTION_RX_SOP                                         0x000C /* RX_SOP */
#define ENUM_LED_CNTRL_LED1_FUNCTION_ON                                             0x000D /* ON */
#define ENUM_LED_CNTRL_LED1_FUNCTION_OFF                                            0x000E /* OFF */
#define ENUM_LED_CNTRL_LED1_FUNCTION_BLINK                                          0x000F /* BLINK */
#define ENUM_LED_CNTRL_LED1_FUNCTION_TX_LEVEL_2P4                                   0x0010 /* TX_LEVEL_2P4 */
#define ENUM_LED_CNTRL_LED1_FUNCTION_TX_LEVEL_1P0                                   0x0011 /* TX_LEVEL_1P0 */
#define ENUM_LED_CNTRL_LED1_FUNCTION_MASTER                                         0x0012 /* MASTER */
#define ENUM_LED_CNTRL_LED1_FUNCTION_SLAVE                                          0x0013 /* SLAVE */
#define ENUM_LED_CNTRL_LED1_FUNCTION_INCOMPATIBLE_LINK_CFG                          0x0014 /* INCOMPATIBLE_LINK_CFG */
#define ENUM_LED_CNTRL_LED1_FUNCTION_AN_LINK_GOOD                                   0x0015 /* AN_LINK_GOOD */
#define ENUM_LED_CNTRL_LED1_FUNCTION_AN_COMPLETE                                    0x0016 /* AN_COMPLETE */
#define ENUM_LED_CNTRL_LED1_FUNCTION_TS_TIMER                                       0x0017 /* TS_TIMER */
#define ENUM_LED_CNTRL_LED1_FUNCTION_LOC_RCVR_STATUS                                0x0018 /* LOC_RCVR_STATUS */
#define ENUM_LED_CNTRL_LED1_FUNCTION_REM_RCVR_STATUS                                0x0019 /* REM_RCVR_STATUS */
#define ENUM_LED_CNTRL_LED1_FUNCTION_CLK25_REF                                      0x001A /* CLK25_REF */
#define ENUM_LED_CNTRL_LED1_FUNCTION_TX_TCLK                                        0x001B /* TX_TCLK */
#define ENUM_LED_CNTRL_LED1_FUNCTION_CLK_120MHZ                                     0x001C /* CLK_120MHZ */
#define ENUM_LED_CNTRL_LED0_MODE_LED_MODE1                                          0x0000 /* LED_MODE1 */
#define ENUM_LED_CNTRL_LED0_MODE_LED_MODE2                                          0x0001 /* LED_MODE2 */
#define ENUM_LED_CNTRL_LED0_FUNCTION_LINKUP_TXRX_ACTIVITY                           0x0000 /* LINKUP_TXRX_ACTIVITY */
#define ENUM_LED_CNTRL_LED0_FUNCTION_LINKUP_TX_ACTIVITY                             0x0001 /* LINKUP_TX_ACTIVITY */
#define ENUM_LED_CNTRL_LED0_FUNCTION_LINKUP_RX_ACTIVITY                             0x0002 /* LINKUP_RX_ACTIVITY */
#define ENUM_LED_CNTRL_LED0_FUNCTION_LINKUP_ONLY                                    0x0003 /* LINKUP_ONLY */
#define ENUM_LED_CNTRL_LED0_FUNCTION_TXRX_ACTIVITY                                  0x0004 /* TXRX_ACTIVITY */
#define ENUM_LED_CNTRL_LED0_FUNCTION_TX_ACTIVITY                                    0x0005 /* TX_ACTIVITY */
#define ENUM_LED_CNTRL_LED0_FUNCTION_RX_ACTIVITY                                    0x0006 /* RX_ACTIVITY */
#define ENUM_LED_CNTRL_LED0_FUNCTION_LINKUP_RX_ER                                   0x0007 /* LINKUP_RX_ER */
#define ENUM_LED_CNTRL_LED0_FUNCTION_LINKUP_RX_TX_ER                                0x0008 /* LINKUP_RX_TX_ER */
#define ENUM_LED_CNTRL_LED0_FUNCTION_RX_ER                                          0x0009 /* RX_ER */
#define ENUM_LED_CNTRL_LED0_FUNCTION_RX_TX_ER                                       0x000A /* RX_TX_ER */
#define ENUM_LED_CNTRL_LED0_FUNCTION_TX_SOP                                         0x000B /* TX_SOP */
#define ENUM_LED_CNTRL_LED0_FUNCTION_RX_SOP                                         0x000C /* RX_SOP */
#define ENUM_LED_CNTRL_LED0_FUNCTION_ON                                             0x000D /* ON */
#define ENUM_LED_CNTRL_LED0_FUNCTION_OFF                                            0x000E /* OFF */
#define ENUM_LED_CNTRL_LED0_FUNCTION_BLINK                                          0x000F /* BLINK */
#define ENUM_LED_CNTRL_LED0_FUNCTION_TX_LEVEL_2P4                                   0x0010 /* TX_LEVEL_2P4 */
#define ENUM_LED_CNTRL_LED0_FUNCTION_TX_LEVEL_1P0                                   0x0011 /* TX_LEVEL_1P0 */
#define ENUM_LED_CNTRL_LED0_FUNCTION_MASTER                                         0x0012 /* MASTER */
#define ENUM_LED_CNTRL_LED0_FUNCTION_SLAVE                                          0x0013 /* SLAVE */
#define ENUM_LED_CNTRL_LED0_FUNCTION_INCOMPATIBLE_LINK_CFG                          0x0014 /* INCOMPATIBLE_LINK_CFG */
#define ENUM_LED_CNTRL_LED0_FUNCTION_AN_LINK_GOOD                                   0x0015 /* AN_LINK_GOOD */
#define ENUM_LED_CNTRL_LED0_FUNCTION_AN_COMPLETE                                    0x0016 /* AN_COMPLETE */
#define ENUM_LED_CNTRL_LED0_FUNCTION_TS_TIMER                                       0x0017 /* TS_TIMER */
#define ENUM_LED_CNTRL_LED0_FUNCTION_LOC_RCVR_STATUS                                0x0018 /* LOC_RCVR_STATUS */
#define ENUM_LED_CNTRL_LED0_FUNCTION_REM_RCVR_STATUS                                0x0019 /* REM_RCVR_STATUS */
#define ENUM_LED_CNTRL_LED0_FUNCTION_CLK25_REF                                      0x001A /* CLK25_REF */
#define ENUM_LED_CNTRL_LED0_FUNCTION_TX_TCLK                                        0x001B /* TX_TCLK */
#define ENUM_LED_CNTRL_LED0_FUNCTION_CLK_120MHZ                                     0x001C /* CLK_120MHZ */

/* ----------------------------------------------------------------------------------------------------
          LED_POLARITY
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LED_POLARITY_LED0_POLARITY                                             0 /* LED 0 Polarity */
#define BITL_LED_POLARITY_LED0_POLARITY                                             2 /* LED 0 Polarity */
#define BITM_LED_POLARITY_LED0_POLARITY                                             0x0003 /* LED 0 Polarity */
#define BITP_LED_POLARITY_LED1_POLARITY                                             2 /* LED 1 Polarity */
#define BITL_LED_POLARITY_LED1_POLARITY                                             2 /* LED 1 Polarity */
#define BITM_LED_POLARITY_LED1_POLARITY                                             0x000C /* LED 1 Polarity */

#define ENUM_LED_POLARITY_LED1_POLARITY_LED_AUTO_SENSE                              0x0000 /* LED Auto Sense */
#define ENUM_LED_POLARITY_LED1_POLARITY_LED_ACTIVE_HI                               0x0001 /* LED Active High */
#define ENUM_LED_POLARITY_LED1_POLARITY_LED_ACTIVE_LO                               0x0002 /* LED Active Low */
#define ENUM_LED_POLARITY_LED0_POLARITY_LED_AUTO_SENSE                              0x0000 /* LED Auto Sense */
#define ENUM_LED_POLARITY_LED0_POLARITY_LED_ACTIVE_HI                               0x0001 /* LED Active High */
#define ENUM_LED_POLARITY_LED0_POLARITY_LED_ACTIVE_LO                               0x0002 /* LED Active Low */

/* ----------------------------------------------------------------------------------------------------
          MMD2_DEV_ID1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD2_DEV_ID1_MMD2_DEV_ID1                                              0 /* Vendor specific 2 MMD device identifier */
#define BITL_MMD2_DEV_ID1_MMD2_DEV_ID1                                              16 /* Vendor specific 2 MMD device identifier */
#define BITM_MMD2_DEV_ID1_MMD2_DEV_ID1                                              0xFFFF /* Vendor specific 2 MMD device identifier */

/* ----------------------------------------------------------------------------------------------------
          MMD2_DEV_ID2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD2_DEV_ID2_MMD2_REV_NUM                                              0 /* Revision number */
#define BITL_MMD2_DEV_ID2_MMD2_REV_NUM                                              4 /* Revision number */
#define BITM_MMD2_DEV_ID2_MMD2_REV_NUM                                              0x000F /* Revision number */
#define BITP_MMD2_DEV_ID2_MMD2_MODEL_NUM                                            4 /* Model Number */
#define BITL_MMD2_DEV_ID2_MMD2_MODEL_NUM                                            6 /* Model Number */
#define BITM_MMD2_DEV_ID2_MMD2_MODEL_NUM                                            0x03F0 /* Model Number */
#define BITP_MMD2_DEV_ID2_MMD2_DEV_ID2_OUI                                          10 /* OUI bits */
#define BITL_MMD2_DEV_ID2_MMD2_DEV_ID2_OUI                                          6 /* OUI bits */
#define BITM_MMD2_DEV_ID2_MMD2_DEV_ID2_OUI                                          0xFC00 /* OUI bits */

/* ----------------------------------------------------------------------------------------------------
          MMD2_DEVS_IN_PKG1
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD2_DEVS_IN_PKG1_MMD2_DEVS_IN_PKG1                                    0 /* Vendor Specific 2 MMD Devices in Package */
#define BITL_MMD2_DEVS_IN_PKG1_MMD2_DEVS_IN_PKG1                                    16 /* Vendor Specific 2 MMD Devices in Package */
#define BITM_MMD2_DEVS_IN_PKG1_MMD2_DEVS_IN_PKG1                                    0xFFFF /* Vendor Specific 2 MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          MMD2_DEVS_IN_PKG2
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD2_DEVS_IN_PKG2_MMD2_DEVS_IN_PKG2                                    0 /* Vendor Specific 2 MMD Devices in Package */
#define BITL_MMD2_DEVS_IN_PKG2_MMD2_DEVS_IN_PKG2                                    16 /* Vendor Specific 2 MMD Devices in Package */
#define BITM_MMD2_DEVS_IN_PKG2_MMD2_DEVS_IN_PKG2                                    0xFFFF /* Vendor Specific 2 MMD Devices in Package */

/* ----------------------------------------------------------------------------------------------------
          MMD2_STATUS
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MMD2_STATUS_MMD2_STATUS                                                14 /* Vendor specific 2 MMD Status */
#define BITL_MMD2_STATUS_MMD2_STATUS                                                2 /* Vendor specific 2 MMD Status */
#define BITM_MMD2_STATUS_MMD2_STATUS                                                0xC000 /* Vendor specific 2 MMD Status */

/* ----------------------------------------------------------------------------------------------------
          PHY_SUBSYS_IRQ_STATUS
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PHY_SUBSYS_IRQ_STATUS_LINK_STAT_CHNG_LH                                1 /* Link Status Change. */
#define BITL_PHY_SUBSYS_IRQ_STATUS_LINK_STAT_CHNG_LH                                1 /* Link Status Change. */
#define BITM_PHY_SUBSYS_IRQ_STATUS_LINK_STAT_CHNG_LH                                0x0002 /* Link Status Change. */
#define BITP_PHY_SUBSYS_IRQ_STATUS_AN_STAT_CHNG_IRQ_LH                              11 /* Autonegotiation Status Change Interrupt. */
#define BITL_PHY_SUBSYS_IRQ_STATUS_AN_STAT_CHNG_IRQ_LH                              1 /* Autonegotiation Status Change Interrupt. */
#define BITM_PHY_SUBSYS_IRQ_STATUS_AN_STAT_CHNG_IRQ_LH                              0x0800 /* Autonegotiation Status Change Interrupt. */
#define BITP_PHY_SUBSYS_IRQ_STATUS_MAC_IF_EBUF_ERR_IRQ_LH                           13 /* Mac Interface Buffers Overflow/underflow Interrupt */
#define BITL_PHY_SUBSYS_IRQ_STATUS_MAC_IF_EBUF_ERR_IRQ_LH                           1 /* Mac Interface Buffers Overflow/underflow Interrupt */
#define BITM_PHY_SUBSYS_IRQ_STATUS_MAC_IF_EBUF_ERR_IRQ_LH                           0x2000 /* Mac Interface Buffers Overflow/underflow Interrupt */
#define BITP_PHY_SUBSYS_IRQ_STATUS_MAC_IF_FC_FG_IRQ_LH                              14 /* Mac Interface Frame CHECKER/_GENERATOR Interrupt */
#define BITL_PHY_SUBSYS_IRQ_STATUS_MAC_IF_FC_FG_IRQ_LH                              1 /* Mac Interface Frame CHECKER/_GENERATOR Interrupt */
#define BITM_PHY_SUBSYS_IRQ_STATUS_MAC_IF_FC_FG_IRQ_LH                              0x4000 /* Mac Interface Frame CHECKER/_GENERATOR Interrupt */

/* ----------------------------------------------------------------------------------------------------
          PHY_SUBSYS_IRQ_MASK
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PHY_SUBSYS_IRQ_MASK_LINK_STAT_CHNG_IRQ_EN                              1 /* Enable Link Status Change Interrupt. */
#define BITL_PHY_SUBSYS_IRQ_MASK_LINK_STAT_CHNG_IRQ_EN                              1 /* Enable Link Status Change Interrupt. */
#define BITM_PHY_SUBSYS_IRQ_MASK_LINK_STAT_CHNG_IRQ_EN                              0x0002 /* Enable Link Status Change Interrupt. */
#define BITP_PHY_SUBSYS_IRQ_MASK_AN_STAT_CHNG_IRQ_EN                                11 /* Enable Autonegotiation Status Change Interrupt. */
#define BITL_PHY_SUBSYS_IRQ_MASK_AN_STAT_CHNG_IRQ_EN                                1 /* Enable Autonegotiation Status Change Interrupt. */
#define BITM_PHY_SUBSYS_IRQ_MASK_AN_STAT_CHNG_IRQ_EN                                0x0800 /* Enable Autonegotiation Status Change Interrupt. */
#define BITP_PHY_SUBSYS_IRQ_MASK_MAC_IF_EBUF_ERR_IRQ_EN                             13 /* Enable Mac Interface Buffers Overflow/underflow Interrupt */
#define BITL_PHY_SUBSYS_IRQ_MASK_MAC_IF_EBUF_ERR_IRQ_EN                             1 /* Enable Mac Interface Buffers Overflow/underflow Interrupt */
#define BITM_PHY_SUBSYS_IRQ_MASK_MAC_IF_EBUF_ERR_IRQ_EN                             0x2000 /* Enable Mac Interface Buffers Overflow/underflow Interrupt */
#define BITP_PHY_SUBSYS_IRQ_MASK_MAC_IF_FC_FG_IRQ_EN                                14 /* Enable Mac Interface Frame CHECKER/_GENERATOR Interrupt */
#define BITL_PHY_SUBSYS_IRQ_MASK_MAC_IF_FC_FG_IRQ_EN                                1 /* Enable Mac Interface Frame CHECKER/_GENERATOR Interrupt */
#define BITM_PHY_SUBSYS_IRQ_MASK_MAC_IF_FC_FG_IRQ_EN                                0x4000 /* Enable Mac Interface Frame CHECKER/_GENERATOR Interrupt */

/* ----------------------------------------------------------------------------------------------------
          FC_EN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_EN_FC_EN                                                            0 /* Frame Checker Enable */
#define BITL_FC_EN_FC_EN                                                            1 /* Frame Checker Enable */
#define BITM_FC_EN_FC_EN                                                            0x0001 /* Frame Checker Enable */

/* ----------------------------------------------------------------------------------------------------
          FC_IRQ_EN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_IRQ_EN_FC_IRQ_EN                                                    0 /* Frame Checker Interrupt Enable */
#define BITL_FC_IRQ_EN_FC_IRQ_EN                                                    1 /* Frame Checker Interrupt Enable */
#define BITM_FC_IRQ_EN_FC_IRQ_EN                                                    0x0001 /* Frame Checker Interrupt Enable */

/* ----------------------------------------------------------------------------------------------------
          FC_TX_SEL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_TX_SEL_FC_TX_SEL                                                    0 /* Frame Checker Transmit Select */
#define BITL_FC_TX_SEL_FC_TX_SEL                                                    1 /* Frame Checker Transmit Select */
#define BITM_FC_TX_SEL_FC_TX_SEL                                                    0x0001 /* Frame Checker Transmit Select */

/* ----------------------------------------------------------------------------------------------------
          RX_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_RX_ERR_CNT_RX_ERR_CNT                                                  0 /* Receive Error Count */
#define BITL_RX_ERR_CNT_RX_ERR_CNT                                                  16 /* Receive Error Count */
#define BITM_RX_ERR_CNT_RX_ERR_CNT                                                  0xFFFF /* Receive Error Count */

/* ----------------------------------------------------------------------------------------------------
          FC_FRM_CNT_H
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_FRM_CNT_H_FC_FRM_CNT_H                                              0 /* Bits [31:16] of Latched Copy of the Number of Received Frames */
#define BITL_FC_FRM_CNT_H_FC_FRM_CNT_H                                              16 /* Bits [31:16] of Latched Copy of the Number of Received Frames */
#define BITM_FC_FRM_CNT_H_FC_FRM_CNT_H                                              0xFFFF /* Bits [31:16] of Latched Copy of the Number of Received Frames */

/* ----------------------------------------------------------------------------------------------------
          FC_FRM_CNT_L
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_FRM_CNT_L_FC_FRM_CNT_L                                              0 /* Bits [15:0] of Latched Copy of the Number of Received Frames */
#define BITL_FC_FRM_CNT_L_FC_FRM_CNT_L                                              16 /* Bits [15:0] of Latched Copy of the Number of Received Frames */
#define BITM_FC_FRM_CNT_L_FC_FRM_CNT_L                                              0xFFFF /* Bits [15:0] of Latched Copy of the Number of Received Frames */

/* ----------------------------------------------------------------------------------------------------
          FC_LEN_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_LEN_ERR_CNT_FC_LEN_ERR_CNT                                          0 /* Latched Copy of the Frame Length Error Counter */
#define BITL_FC_LEN_ERR_CNT_FC_LEN_ERR_CNT                                          16 /* Latched Copy of the Frame Length Error Counter */
#define BITM_FC_LEN_ERR_CNT_FC_LEN_ERR_CNT                                          0xFFFF /* Latched Copy of the Frame Length Error Counter */

/* ----------------------------------------------------------------------------------------------------
          FC_ALGN_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_ALGN_ERR_CNT_FC_ALGN_ERR_CNT                                        0 /* Latched Copy of the Frame Alignment Error Counter */
#define BITL_FC_ALGN_ERR_CNT_FC_ALGN_ERR_CNT                                        16 /* Latched Copy of the Frame Alignment Error Counter */
#define BITM_FC_ALGN_ERR_CNT_FC_ALGN_ERR_CNT                                        0xFFFF /* Latched Copy of the Frame Alignment Error Counter */

/* ----------------------------------------------------------------------------------------------------
          FC_SYMB_ERR_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_SYMB_ERR_CNT_FC_SYMB_ERR_CNT                                        0 /* Latched Copy of the Symbol Error Counter */
#define BITL_FC_SYMB_ERR_CNT_FC_SYMB_ERR_CNT                                        16 /* Latched Copy of the Symbol Error Counter */
#define BITM_FC_SYMB_ERR_CNT_FC_SYMB_ERR_CNT                                        0xFFFF /* Latched Copy of the Symbol Error Counter */

/* ----------------------------------------------------------------------------------------------------
          FC_OSZ_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_OSZ_CNT_FC_OSZ_CNT                                                  0 /* Latched copy of the Overisized Frame Error Counter */
#define BITL_FC_OSZ_CNT_FC_OSZ_CNT                                                  16 /* Latched copy of the Overisized Frame Error Counter */
#define BITM_FC_OSZ_CNT_FC_OSZ_CNT                                                  0xFFFF /* Latched copy of the Overisized Frame Error Counter */

/* ----------------------------------------------------------------------------------------------------
          FC_USZ_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_USZ_CNT_FC_USZ_CNT                                                  0 /* Latched Copy of the Undersized Frame Error Counter */
#define BITL_FC_USZ_CNT_FC_USZ_CNT                                                  16 /* Latched Copy of the Undersized Frame Error Counter */
#define BITM_FC_USZ_CNT_FC_USZ_CNT                                                  0xFFFF /* Latched Copy of the Undersized Frame Error Counter */

/* ----------------------------------------------------------------------------------------------------
          FC_ODD_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_ODD_CNT_FC_ODD_CNT                                                  0 /* Latched Copy of the Odd Nibble Counter */
#define BITL_FC_ODD_CNT_FC_ODD_CNT                                                  16 /* Latched Copy of the Odd Nibble Counter */
#define BITM_FC_ODD_CNT_FC_ODD_CNT                                                  0xFFFF /* Latched Copy of the Odd Nibble Counter */

/* ----------------------------------------------------------------------------------------------------
          FC_ODD_PRE_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_ODD_PRE_CNT_FC_ODD_PRE_CNT                                          0 /* Latched Copy of the Odd Preamble Packet Counter */
#define BITL_FC_ODD_PRE_CNT_FC_ODD_PRE_CNT                                          16 /* Latched Copy of the Odd Preamble Packet Counter */
#define BITM_FC_ODD_PRE_CNT_FC_ODD_PRE_CNT                                          0xFFFF /* Latched Copy of the Odd Preamble Packet Counter */

/* ----------------------------------------------------------------------------------------------------
          FC_FALSE_CARRIER_CNT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FC_FALSE_CARRIER_CNT_FC_FALSE_CARRIER_CNT                              0 /* Latched Copy of the False Carrier Events Counter */
#define BITL_FC_FALSE_CARRIER_CNT_FC_FALSE_CARRIER_CNT                              16 /* Latched Copy of the False Carrier Events Counter */
#define BITM_FC_FALSE_CARRIER_CNT_FC_FALSE_CARRIER_CNT                              0xFFFF /* Latched Copy of the False Carrier Events Counter */

/* ----------------------------------------------------------------------------------------------------
          FG_EN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FG_EN_FG_EN                                                            0 /* Frame Generator Enable */
#define BITL_FG_EN_FG_EN                                                            1 /* Frame Generator Enable */
#define BITM_FG_EN_FG_EN                                                            0x0001 /* Frame Generator Enable */

/* ----------------------------------------------------------------------------------------------------
          FG_CNTRL_RSTRT
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FG_CNTRL_RSTRT_FG_CNTRL                                                0 /* Frame Generator Control */
#define BITL_FG_CNTRL_RSTRT_FG_CNTRL                                                3 /* Frame Generator Control */
#define BITM_FG_CNTRL_RSTRT_FG_CNTRL                                                0x0007 /* Frame Generator Control */
#define BITP_FG_CNTRL_RSTRT_FG_RSTRT                                                3 /* Frame Generator Restart */
#define BITL_FG_CNTRL_RSTRT_FG_RSTRT                                                1 /* Frame Generator Restart */
#define BITM_FG_CNTRL_RSTRT_FG_RSTRT                                                0x0008 /* Frame Generator Restart */

#define ENUM_FG_CNTRL_RSTRT_FG_CNTRL_FG_GEN_NONE                                    0x0000 /* No description provided */
#define ENUM_FG_CNTRL_RSTRT_FG_CNTRL_FG_GEN_RANDOM_PAYLOAD                          0x0001 /* No description provided */
#define ENUM_FG_CNTRL_RSTRT_FG_CNTRL_FG_GEN_0x00_PAYLOAD                            0x0002 /* No description provided */
#define ENUM_FG_CNTRL_RSTRT_FG_CNTRL_FG_GEN_0xFF_PAYLOAD                            0x0003 /* No description provided */
#define ENUM_FG_CNTRL_RSTRT_FG_CNTRL_FG_GEN_0x55_PAYLOAD                            0x0004 /* No description provided */
#define ENUM_FG_CNTRL_RSTRT_FG_CNTRL_FG_GEN_DECR_PAYLOAD                            0x0005 /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FG_CONT_MODE_EN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FG_CONT_MODE_EN_FG_CONT_MODE_EN                                        0 /* Frame Generator Continuous Mode Enable */
#define BITL_FG_CONT_MODE_EN_FG_CONT_MODE_EN                                        1 /* Frame Generator Continuous Mode Enable */
#define BITM_FG_CONT_MODE_EN_FG_CONT_MODE_EN                                        0x0001 /* Frame Generator Continuous Mode Enable */

/* ----------------------------------------------------------------------------------------------------
          FG_IRQ_EN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FG_IRQ_EN_FG_IRQ_EN                                                    0 /* Frame Generator Interrupt Enable */
#define BITL_FG_IRQ_EN_FG_IRQ_EN                                                    1 /* Frame Generator Interrupt Enable */
#define BITM_FG_IRQ_EN_FG_IRQ_EN                                                    0x0001 /* Frame Generator Interrupt Enable */

/* ----------------------------------------------------------------------------------------------------
          FG_FRM_LEN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FG_FRM_LEN_FG_FRM_LEN                                                  0 /* The Data Field Frame Length in Bytes */
#define BITL_FG_FRM_LEN_FG_FRM_LEN                                                  16 /* The Data Field Frame Length in Bytes */
#define BITM_FG_FRM_LEN_FG_FRM_LEN                                                  0xFFFF /* The Data Field Frame Length in Bytes */

/* ----------------------------------------------------------------------------------------------------
          FG_IFG_LEN
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FG_IFG_LEN_FG_IFG_LEN                                                  0 /* Frame Generator Inter-frame Gap Length */
#define BITL_FG_IFG_LEN_FG_IFG_LEN                                                  16 /* Frame Generator Inter-frame Gap Length */
#define BITM_FG_IFG_LEN_FG_IFG_LEN                                                  0xFFFF /* Frame Generator Inter-frame Gap Length */

/* ----------------------------------------------------------------------------------------------------
          FG_NFRM_H
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FG_NFRM_H_FG_NFRM_H                                                    0 /* Bits [31:16] of the Number of Frames to be Generated */
#define BITL_FG_NFRM_H_FG_NFRM_H                                                    16 /* Bits [31:16] of the Number of Frames to be Generated */
#define BITM_FG_NFRM_H_FG_NFRM_H                                                    0xFFFF /* Bits [31:16] of the Number of Frames to be Generated */

/* ----------------------------------------------------------------------------------------------------
          FG_NFRM_L
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FG_NFRM_L_FG_NFRM_L                                                    0 /* Bits [15:0] of the Number of Frames to be Generated */
#define BITL_FG_NFRM_L_FG_NFRM_L                                                    16 /* Bits [15:0] of the Number of Frames to be Generated */
#define BITM_FG_NFRM_L_FG_NFRM_L                                                    0xFFFF /* Bits [15:0] of the Number of Frames to be Generated */

/* ----------------------------------------------------------------------------------------------------
          FG_DONE
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FG_DONE_FG_DONE                                                        0 /* Frame Generator Done */
#define BITL_FG_DONE_FG_DONE                                                        1 /* Frame Generator Done */
#define BITM_FG_DONE_FG_DONE                                                        0x0001 /* Frame Generator Done */

/* ----------------------------------------------------------------------------------------------------
          MAC_IF_LOOPBACK
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_IF_LOOPBACK_MAC_IF_LB_EN                                           0 /* MAC Interface Loopback Enable. */
#define BITL_MAC_IF_LOOPBACK_MAC_IF_LB_EN                                           1 /* MAC Interface Loopback Enable. */
#define BITM_MAC_IF_LOOPBACK_MAC_IF_LB_EN                                           0x0001 /* MAC Interface Loopback Enable. */
#define BITP_MAC_IF_LOOPBACK_MAC_IF_LB_TX_SUP_EN                                    1 /* Suppress Transmission Enable. */
#define BITL_MAC_IF_LOOPBACK_MAC_IF_LB_TX_SUP_EN                                    1 /* Suppress Transmission Enable. */
#define BITM_MAC_IF_LOOPBACK_MAC_IF_LB_TX_SUP_EN                                    0x0002 /* Suppress Transmission Enable. */
#define BITP_MAC_IF_LOOPBACK_MAC_IF_REM_LB_EN                                       2 /* MAC Interface Remote Loopback Enable. */
#define BITL_MAC_IF_LOOPBACK_MAC_IF_REM_LB_EN                                       1 /* MAC Interface Remote Loopback Enable. */
#define BITM_MAC_IF_LOOPBACK_MAC_IF_REM_LB_EN                                       0x0004 /* MAC Interface Remote Loopback Enable. */
#define BITP_MAC_IF_LOOPBACK_MAC_IF_REM_LB_RX_SUP_EN                                3 /* Suppress RX Enable */
#define BITL_MAC_IF_LOOPBACK_MAC_IF_REM_LB_RX_SUP_EN                                1 /* Suppress RX Enable */
#define BITM_MAC_IF_LOOPBACK_MAC_IF_REM_LB_RX_SUP_EN                                0x0008 /* Suppress RX Enable */

/* ----------------------------------------------------------------------------------------------------
          MAC_IF_SOP_CNTRL
   ---------------------------------------------------------------------------------------------------- */
#define BITP_MAC_IF_SOP_CNTRL_MAC_IF_RX_SOP_DET_EN                                  0 /* Enable the Generation of the Rx SOP Indication Signal. */
#define BITL_MAC_IF_SOP_CNTRL_MAC_IF_RX_SOP_DET_EN                                  1 /* Enable the Generation of the Rx SOP Indication Signal. */
#define BITM_MAC_IF_SOP_CNTRL_MAC_IF_RX_SOP_DET_EN                                  0x0001 /* Enable the Generation of the Rx SOP Indication Signal. */
#define BITP_MAC_IF_SOP_CNTRL_MAC_IF_RX_SOP_SFD_EN                                  1 /* Enable Rx SOP Signal Indication on SFD Reception. */
#define BITL_MAC_IF_SOP_CNTRL_MAC_IF_RX_SOP_SFD_EN                                  1 /* Enable Rx SOP Signal Indication on SFD Reception. */
#define BITM_MAC_IF_SOP_CNTRL_MAC_IF_RX_SOP_SFD_EN                                  0x0002 /* Enable Rx SOP Signal Indication on SFD Reception. */
#define BITP_MAC_IF_SOP_CNTRL_MAC_IF_RX_SOP_LEN_CHK_EN                              2 /* Enable Rx SOP Preamble Length Check. */
#define BITL_MAC_IF_SOP_CNTRL_MAC_IF_RX_SOP_LEN_CHK_EN                              1 /* Enable Rx SOP Preamble Length Check. */
#define BITM_MAC_IF_SOP_CNTRL_MAC_IF_RX_SOP_LEN_CHK_EN                              0x0004 /* Enable Rx SOP Preamble Length Check. */
#define BITP_MAC_IF_SOP_CNTRL_MAC_IF_TX_SOP_DET_EN                                  3 /* Enable the Generation of the Tx SOP Indication Signal. */
#define BITL_MAC_IF_SOP_CNTRL_MAC_IF_TX_SOP_DET_EN                                  1 /* Enable the Generation of the Tx SOP Indication Signal. */
#define BITM_MAC_IF_SOP_CNTRL_MAC_IF_TX_SOP_DET_EN                                  0x0008 /* Enable the Generation of the Tx SOP Indication Signal. */
#define BITP_MAC_IF_SOP_CNTRL_MAC_IF_TX_SOP_SFD_EN                                  4 /* Enable Tx SOP Signal Indication on SFD. */
#define BITL_MAC_IF_SOP_CNTRL_MAC_IF_TX_SOP_SFD_EN                                  1 /* Enable Tx SOP Signal Indication on SFD. */
#define BITM_MAC_IF_SOP_CNTRL_MAC_IF_TX_SOP_SFD_EN                                  0x0010 /* Enable Tx SOP Signal Indication on SFD. */
#define BITP_MAC_IF_SOP_CNTRL_MAC_IF_TX_SOP_LEN_CHK_EN                              5 /* Enable Tx SOP Preamble Length Check. */
#define BITL_MAC_IF_SOP_CNTRL_MAC_IF_TX_SOP_LEN_CHK_EN                              1 /* Enable Tx SOP Preamble Length Check. */
#define BITM_MAC_IF_SOP_CNTRL_MAC_IF_TX_SOP_LEN_CHK_EN                              0x0020 /* Enable Tx SOP Preamble Length Check. */

#define ADIN2111_MAC_SPI_PROT_GENERIC                                               0x01
#define ADIN2111_MAC_SPI_FULL_DUPLEX                                                0x01
#define ADIN2111_MAC_SPI_HALF_DUPLEX                                                0x00
#define ADIN2111_SPI_HEADER_SIZE                                                    0x02
#define ADIN2111_FRAME_HEADER_SIZE                                                  0x02
#define ADIN2111_TURNAROUND_SIZE                                                    0x01
#define ADIN2111_SPI_TX_FIFO_BUFFER                                                 0x04

#define ADIN2111_MAC_SPI_TRANSACTION_DATA                                           0x00
#define ADIN2111_MAC_SPI_TRANSACTION_CONTROL                                        0x01
#define ADIN2111_MAC_SPI_READ                                                       0x00
#define ADIN2111_MAC_SPI_WRITE                                                      0x01

/* 8-bit data used to test CRC Data fill mode */
#define ADIN2111_CRC8CCITT_POLYNOMIAL_BE                                            0x07
#define ADIN2111_CRC8CCITT_POLYNOMIAL_LE                                            0xE0
/* 8-bit data used to test CRC Data fill mode */
#define ADIN2111_CRC8_SEED_VALUE                                                    0x00
/*! SPI register access size in bytes. */
#define ADIN2111_MAC_SPI_ACCESS_SIZE                                                0x04
/* Random MAC generation 
   TODO: Move to generating from 96bit UID */
#define ADIN2111_MAC_PHYID                                                          0x283BCA1

/*! Maximum number of attempts to establish the read the expected value of PHYID register during the initialization. */
/*  This is based on a wait time of 50ms and a SPI read of 2us.                                                      */
#define ADIN2111_MAC_INIT_MAX_RETRIES                                               25000

/*! Maximum number of attempts to establish the SPI interface is up and responsive. */
#define ADIN2111_MAC_IF_UP_MAX_RETRIES                                              25000

/*! Hardware reset value of MMD1_DEV_ID1 register, used for device identification. */
#define ADIN2111_PHY_DEVID1                                                         0x0283

/*! Hardware reset value of MMD1_DEV_ID2 register (OUI field), used for device identification. */
#define ADIN2111_PHY_DEVID2_OUI                                                     0x2F

/*! Number of retries allowed for software powerdown entry, in MDIO read count. */
#define ADIN2111_PHY_SOFT_PD_ITER                                                   10

/*! Maximum number of attempts to read MDIO RDY bit. */
#define ADIN2111_MAC_MDIO_MAX_RETRIES                                               10

/*! CRSM interrupt sources showing a hardware error that requires a reset/reconfiguration of the device. */
#define ADIN2111_PHY_CRSM_HW_ERROR                                                  0x2BFF

/* MAC reset keys */
/*! Key 1 for MAC-only reset. */
#define RST_MAC_ONLY_KEY1                                                           0x4F1C
/*! Key 2 for MAC-only reset. */
#define RST_MAC_ONLY_KEY2                                                           0xC1F4

typedef enum
{
    ADIN2111_ETH_LINK_STATUS_DOWN = (0), /*!< Link down.  */
    ADIN2111_ETH_LINK_STATUS_UP = (1), /*!< Link up.    */
} adin2111_eth_link_status_t;

typedef enum
{
    ADIN2111_ETH_RESET_TYPE_MAC_ONLY = 0, /*!< Reset the MAC only. */
    ADIN2111_ETH_RESET_TYPE_MAC_PHY, /*!< Reset both MAC and PHY. */
} adin2111_eth_reset_type_t;

typedef enum
{
    ADI_MAC_RX_FIFO_PRIO_LOW = 0,
    ADI_MAC_RX_FIFO_PRIO_HIGH,
} adi_mac_rx_fifo_prio_t;

typedef enum
{
    ADI_MAC_EGRESS_CAPTURE_NONE = 0, /*!< No action. */
    ADI_MAC_EGRESS_CAPTURE_A, /*!< Capture egress timestamp A. */
    ADI_MAC_EGRESS_CAPTURE_B, /*!< Capture egress timestamp B. */
    ADI_MAC_EGRESS_CAPTURE_C, /*!< Capture egress timestamp C. */
} adi_mac_egress_capture_t;

typedef enum
{
    ADIN2111_ETH_SUCCESS = 0,                /*!< Success.                                                  */
    ADIN2111_ETH_MDIO_TIMEOUT,               /*!< MDIO timeout.                                             */
    ADIN2111_ETH_COMM_ERROR,                 /*!< Communication error.                                      */
    ADIN2111_ETH_COMM_ERROR_SECOND,          /*!< Communication error.                                      */
    ADIN2111_ETH_COMM_TIMEOUT,               /*!< Communications timeout with the host.                     */
    ADIN2111_ETH_UNSUPPORTED_DEVICE,         /*!< Unsupported device.                                       */
    ADIN2111_ETH_DEVICE_UNINITIALIZED,       /*!< Device not initialized.                                   */
    ADIN2111_ETH_HW_ERROR,                   /*!< Hardware error.                                           */
    ADIN2111_ETH_INVALID_PARAM,              /*!< Invalid parameter.                                        */
    ADIN2111_ETH_PARAM_OUT_OF_RANGE,         /*!< Parameter out of range.                                   */
    ADIN2111_ETH_INVALID_HANDLE,             /*!< Invalid device handle.                                    */
    ADIN2111_ETH_IRQ_PENDING,                /*!< Interrupt request is pending.                             */
    ADIN2111_ETH_READ_STATUS_TIMEOUT,        /*!< Timeout when reading status registers.                    */
    ADIN2111_ETH_INVALID_POWER_STATE,        /*!< Invalid power state.                                      */
    ADIN2111_ETH_HAL_INIT_ERROR,             /*!< HAL initialization error.                                 */
    ADIN2111_ETH_INSUFFICIENT_FIFO_SPACE,    /*!< Insufficient TxFIFO space when trying to write a frame.   */
    ADIN2111_ETH_CRC_ERROR,                  /*!< SPI integrity check failure (generic SPI).                */
    ADIN2111_ETH_PROTECTION_ERROR,           /*!< SPI integrity check failure (OPEN Alliance SPI).          */
    ADIN2111_ETH_QUEUE_FULL,                 /*!< Transmit queue is full.                                   */
    ADIN2111_ETH_QUEUE_EMPTY,                /*!< Receive queue is empty.                                   */
    ADIN2111_ETH_BUFFER_TOO_SMALL,           /*!< Buffer is too small for received data.                    */
    ADIN2111_ETH_INVALID_PORT,               /*!< Invalid port value.                                       */
    ADIN2111_ETH_ADDRESS_FILTER_TABLE_FULL,  /*!< Address filter table is full.                             */
    ADIN2111_ETH_MAC_BUSY,                   /*!< MAC is busy.                                              */
    ADIN2111_ETH_COMM_BUSY,                  /*!< SPI communication busy.                                   */
    ADIN2111_ETH_SPI_ERROR,                  /*!< SPI error.                                                */
    ADIN2111_ETH_SPI_LEN_ERROR,              /*!< Odd Length provided to SPI Write                          */
    ADIN2111_ETH_SW_RESET_TIMEOUT,           /*!< Software reset timeout.                                   */
    ADIN2111_ETH_CONFIG_SYNC_ERROR,          /*!< Configuration change attempted after configuration sync.  */
    ADIN2111_ETH_VALUE_MISMATCH_ERROR,       /*!< Value does not match expected value.                      */
    ADIN2111_ETH_FIFO_SIZE_ERROR,            /*!< Desired FIFO size exceeds 28k byte limit.                 */
    ADIN2111_ETH_TS_COUNTERS_DISABLED,       /*!< Timestamp counters are not enabled.                       */
    ADIN2111_ETH_NO_TS_FORMAT,               /*!< No timstamp format selected or timestamps captured.       */
    ADIN2111_ETH_NOT_IMPLEMENTED,            /*!< Not implemented in hardware.                              */
    ADIN2111_ETH_NOT_IMPLEMENTED_SOFTWARE,   /*!< Not implemented in software.                              */
    ADIN2111_ETH_UNSUPPORTED_FEATURE,        /*!< Hardware feature not supported by the software driver.    */
    ADIN2111_ETH_PLACEHOLDER_ERROR,          /*!< Unassigned (placeholder) error.                           */
} adin2111_return_code_t;

typedef enum {
   ADIN2111_ETH_PORT_1 = 1,
   ADIN2111_ETH_PORT_2 = 2,
} adin2111_port_t;

typedef struct {
    union {
        struct {
             uint16_t ADDR : 13; /*!< SPI Register Address.          */
             uint16_t RW   : 1;  /*!< 0 => Read, 1 => Write          */
             uint16_t FD   : 1;  /*!< Enable Full Duplex.            */
             uint16_t CD   : 1;  /*!< 0 => Data, 1 => Control        */
        };
        uint16_t VALUE16;
    };
} adin2111_mac_spi_header_t;

typedef struct {
    union {
        struct {
             uint32_t MDIO_DATA        : 16; /**< Data/Address Value. */
             uint32_t MDIO_DEVAD       : 5; /**< MDIO Device Address. */
             uint32_t MDIO_PRTAD       : 5; /**< MDIO Port Address. */
             uint32_t MDIO_OP          : 2; /**< Operation Code. */
             uint32_t MDIO_ST          : 2; /**< Start of Frame. */
             uint32_t MDIO_TAERR       : 1; /**< Turnaround Error. */
             uint32_t MDIO_TRDONE      : 1; /**< Transaction Done. */
        };
        uint32_t VALUE32;
    };
} adin2111_mac_mdio_t;

typedef struct {
    union {
        struct {
             uint32_t TXPE             : 1; /**< Transmit Protocol Error. */
             uint32_t TXBOE            : 1; /**< Host Tx FIFO Overflow. */
             uint32_t TXBUE            : 1; /**< Host Tx FIFO Underrun Error. */
             uint32_t RXBOE            : 1; /**< Receive Buffer Overflow Error. */
             uint32_t LOFE             : 1; /**< Loss of Frame Error. */
             uint32_t HDRE             : 1; /**< Header Error. */
             uint32_t RESETC           : 1; /**< Reset Complete. */
             uint32_t PHYINT           : 1; /**< PHY Interrupt for Port1. */
             uint32_t TTSCAA           : 1; /**< Transmit Timestamp Capture Available A. */
             uint32_t TTSCAB           : 1; /**< Transmit Timestamp Capture Available B. */
             uint32_t TTSCAC           : 1; /**< Transmit Timestamp Capture Available C. */
             uint32_t TXFCSE           : 1; /**< Transmit Frame Check Sequence Error. */
             uint32_t CDPE             : 1; /**< Control Data Protection Error. */
             uint32_t RESERVED13       : 19; /**< Reserved */
        };
        uint32_t VALUE32;
    };
} adin2111_mac_status0_t;

typedef struct {
    union {
        struct {
             uint32_t P1_LINK_STATUS   : 1; /**< Port 1 Link Status. */
             uint32_t LINK_CHANGE      : 1; /**< Link Status Changed. */
             uint32_t RESERVED2        : 1; /**< Reserved */
             uint32_t TX_RDY           : 1; /**< Tx Ready. */
             uint32_t P1_RX_RDY        : 1; /**< Port 1 Rx FIFO Contains Data. */
             uint32_t P1_RX_RDY_HI     : 1; /**< Port1 Rx Ready High Priority. */
             uint32_t RESERVED6        : 2; /**< Reserved */
             uint32_t P1_RX_IFG_ERR    : 1; /**< Rx MAC Inter Frame Gap Error. */
             uint32_t RESERVED9        : 1; /**< Reserved */
             uint32_t SPI_ERR          : 1; /**< Detected an Error on an SPI Transaction. */
             uint32_t RX_ECC_ERR       : 1; /**< ECC Error on Reading the Frame Size from an Rx FIFO. */
             uint32_t TX_ECC_ERR       : 1; /**< ECC Error on Reading the Frame Size from a Tx FIFO. */
             uint32_t RESERVED13       : 19; /**< Reserved */
        };
        uint32_t VALUE32;
    };
} adin2111_mac_status1_t;

typedef struct
{
    union {
        struct {
            uint16_t PORT               : 1;
            uint16_t RSVD0              : 1;
            uint16_t TIME_STAMP_PRESENT : 1;
            uint16_t TIME_STAMP_PARITY  : 1;
            uint16_t RSVD1              : 2;
            uint16_t EGRESS_CAPTURE     : 2;
            uint16_t RSVD2              : 2;
            uint16_t PRI                : 1;
            uint16_t RSVD3              : 5;
        };
        uint16_t VALUE16;
    };
} adin2111_mac_frame_header_t;

typedef struct {
    union {
        struct {
             unsigned int TO_HOST          : 1; /**< Forward Frames Matching This MAC Address to the Host. */
             unsigned int TO_OTHER_PORT    : 1; /**< Forward Frames Matching This MAC Address to the Other Port. */
             unsigned int RESERVED2        : 1; /**< Reserved */
             unsigned int HOST_PRI         : 1; /**< Host Rx Port Priority */
             unsigned int RESERVED4        : 10; /**< Reserved */
             unsigned int APPLY2PORT1      : 1; /**< Apply to Port 1. */
             unsigned int APPLY2PORT2      : 1; /**< Apply to Port 2. */
        };
        uint16_t VALUE16;
    };
} adi_mac_address_rule_t;

typedef struct
{
    uint8_t *p_buf; /*!< Pointer to the frame buffer for Tx or Rx. */
    uint32_t buf_size; /*!< Buffer size in bytes, used to check for overflows in Rx. */
    uint32_t trx_size; /*!< Frame size (Tx or Rx), in bytes. */
    bool timestamp_valid; /*!< Indicates if the timestamp field is valid. */
    uint32_t timestamp; /*!< Ingress timestamp received from the MAC. */
    uint32_t timestamp_ext; /*!< Extended storage for 64b ingress timestamp. */
    uint32_t ref_count; /*!< Reference counter, indicates if a descriptor needs to be sent more than once. */
} adin2111_eth_buf_desc_t;

typedef struct
{
    adin2111_mac_frame_header_t header; /*!< Frame header. */
    adin2111_eth_buf_desc_t *p_buf_desc; /*!< Pointer to the buffer descriptor. */
} adin2111_mac_frame_t;

/*! MDIO Device Address extraction from 32-bit PHY register address. */
#define DEVTYPE(a) (a >> 16)
/*! MDIO Register Address extraction from 32-bit PHY register address. */
#define REGADDR(a) (a & 0xFFFF)

#define SPE_NUM_BUFS = 4;
#define SPE_FRAME_SIZE = 1518;
/* Extra 4 bytes for FCS and 2 bytes for the frame header */
#define SPE_MAX_BUF_FRAME_SIZE = (SPE_FRAME_SIZE + 4 + 2);
#define SPE_MIN_PAYLOAD_SIZE = (46);

#define SPE_MAC_SIZE = 6;
#define SPE_FRAME_HEADER_SIZE = (2*SPE_MAC_SIZE + 2);

#define ADIN_MAC_ADDR_0 (0x00)
#define ADIN_MAC_ADDR_1 (0x0A)
#define ADIN_MAC_ADDR_2 (0x83)

/*! Number of buffer bytes in TxFIFO to provide a margin for the frame writes. */
#define ADI_SPI_TX_FIFO_PADDING (4)

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
    K_KERNEL_STACK_MEMBER(thread_stack,
                  CONFIG_ETH_ADIN2111_RX_THREAD_STACK_SIZE);
    struct k_thread thread;
    uint8_t mac_addr[6];
    struct gpio_callback gpio_cb;
    struct k_sem tx_sem;
    struct k_sem int_sem;
    void (*generate_mac)(uint8_t *mac);
    uint8_t buf[NET_ETH_MAX_FRAME_SIZE];
};


#endif /* __ETH_ADIN2111_PRIV_H__ */