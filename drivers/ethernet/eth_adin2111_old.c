#define DT_DRV_COMPAT analog_adin2111

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_adin2111, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <ethernet/eth_stats.h>
#include "eth.h"
#include "eth_adin2111_old_priv.h"
#include <zephyr/random/rand32.h>

static adin2111_return_code_t adin2111_spi_write(const struct device *dev, uint32_t addr, uint8_t *data, uint32_t len);
static adin2111_return_code_t adin2111_spi_read(const struct device *dev, uint32_t addr, uint8_t *data, uint32_t len);
static adin2111_return_code_t adin2111_spi_phy_write(const struct device *dev, uint32_t reg_addr, uint16_t data, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_spi_phy_read(const struct device *dev, uint32_t reg_addr, uint16_t *reg_data, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_write_frame(const struct device *dev, adin2111_mac_frame_t *frame);
static adin2111_return_code_t adin2111_receive_frame(const struct device *dev, uint8_t port);

static adin2111_return_code_t adin2111_phy_static_config(const struct device *dev, uint32_t model_num, uint32_t rev_num, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_phy_get_sw_powerdown(const struct device *dev, bool *enable, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_phy_set_sw_powerdown(const struct device *dev, bool enable, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_phy_check_id(const struct device *dev, uint32_t *model_num, uint32_t *rev_num, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_phy_init(const struct device *dev, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_phy_an_enable(const struct device *dev, bool enable, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_phy_reset(const struct device *dev, adin2111_port_t port_num);

static adin2111_return_code_t adin2111_mac_reset(const struct device *dev, adin2111_eth_reset_type_t reset_type);
static void adin2111_thread(const struct device *dev);
static adin2111_return_code_t adin2111_mac_init(const struct device *dev);
static int adin2111_tx(const struct device *dev, struct net_pkt *pkt);
static void adin2111_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void adin2111_iface_init(struct net_if *iface);
static adin2111_return_code_t adin2111_wait_mdio_ready(const struct device *dev, uint16_t addr_offset);
static enum ethernet_hw_caps adin2111_get_capabilities(const struct device *dev);
static int adin2111_hw_start(const struct device *dev);
static int adin2111_hw_stop(const struct device *dev);
static adin2111_return_code_t adin2111_sync_config(const struct device *dev);
static adin2111_return_code_t adin2111_enable_mac_loopback(const struct device *dev);
static adin2111_return_code_t adin2111_enable_mac_if_loopback(const struct device *dev, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_enable_pcs_loopback(const struct device *dev, adin2111_port_t port_num);
static adin2111_return_code_t adin2111_set_macaddr(const struct device *dev);
static adin2111_return_code_t adin2111_wait_device_ready(const struct device *dev);

static void adin2111_random_mac(uint8_t *mac_addr);
static int adin2111_init(const struct device *dev);

/* ====================================================================================================
        ADIN2111 SPI Read/Write 
   ==================================================================================================== */
static adin2111_return_code_t adin2111_spi_write(const struct device *dev, uint32_t addr,
                                                    uint8_t *data, uint32_t len) {
    const struct adin2111_config *cfg = dev->config;
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    adin2111_mac_spi_header_t spi_hdr;
    uint8_t cmd[2];

    spi_hdr.CD = ADIN2111_MAC_SPI_TRANSACTION_CONTROL;
    spi_hdr.FD = ADIN2111_MAC_SPI_HALF_DUPLEX;
    spi_hdr.RW = ADIN2111_MAC_SPI_WRITE;
    spi_hdr.ADDR = (uint16_t) addr;
    cmd[0] = spi_hdr.VALUE16 >> 8;
    cmd[1] = spi_hdr.VALUE16 & 0xFF;

    const struct spi_buf tx_buf[2] = {
        {
            .buf = cmd,
            .len = ARRAY_SIZE(cmd),
        },
        {
            .buf = data,
            .len = len,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = ARRAY_SIZE(tx_buf),
    };

    /* Confirm len is even */
    if (len & 1) {
        ret = ADIN2111_ETH_SPI_LEN_ERROR;
        goto end;
    }

    if(spi_write_dt(&cfg->spi, &tx) != 0) {
        ret = ADIN2111_ETH_SPI_ERROR;
    }

end:
    return ret;
}

static adin2111_return_code_t adin2111_spi_read(const struct device *dev, uint32_t addr,
              uint8_t *data, uint32_t len) {
    const struct adin2111_config *cfg = dev->config;
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint8_t cmd[ADIN2111_FRAME_HEADER_SIZE];
    adin2111_mac_spi_header_t spi_hdr;

    spi_hdr.CD = ADIN2111_MAC_SPI_TRANSACTION_CONTROL;
    spi_hdr.FD = ADIN2111_MAC_SPI_HALF_DUPLEX;
    spi_hdr.RW = ADIN2111_MAC_SPI_READ;
    spi_hdr.ADDR = (uint16_t) addr;
    cmd[0] = spi_hdr.VALUE16 >> 8;
    cmd[1] = spi_hdr.VALUE16 & 0xFF;

    // Bytes for registers + turnaround
    uint8_t total_offset = ADIN2111_FRAME_HEADER_SIZE + ADIN2111_TURNAROUND_SIZE;
    uint8_t tmp[len + total_offset];

    const struct spi_buf tx_buf = {
        .buf = cmd,
        .len = ARRAY_SIZE(cmd),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };
    const struct spi_buf rx_buf = {
        .buf = tmp,
        .len = ARRAY_SIZE(tmp),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    /* Confirm len is even */
    if (len & 1) {
        ret = ADIN2111_ETH_SPI_LEN_ERROR;
        goto end;
    }

    if (spi_transceive_dt(&cfg->spi, &tx, &rx) != 0) {
        ret = ADIN2111_ETH_SPI_ERROR;
    } else {
        memcpy(data, &tmp[total_offset], len);
    }

end:
    return ret;
}

static adin2111_return_code_t adin2111_spi_phy_write(const struct device *dev, uint32_t reg_addr, uint16_t data, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    adin2111_mac_mdio_t mdio_cmd;

    /* Use the first MDIO register for the address operation. */
    /* MDIO Speed defaults to 2.5MHz per CONFIG2.MSPEED. */
    mdio_cmd.MDIO_DEVAD = DEVTYPE(reg_addr);
    mdio_cmd.MDIO_ST = ENUM_MAC_MDIOACC_N__MDIO_ST_CLAUSE45;
    mdio_cmd.MDIO_PRTAD = port_num;
    mdio_cmd.MDIO_OP = ENUM_MAC_MDIOACC_N__MDIO_OP_MD_ADDR;
    mdio_cmd.MDIO_DATA = REGADDR(reg_addr);
    mdio_cmd.MDIO_TRDONE = 0;

    uint32_t net_mdio_val = htonl(mdio_cmd.VALUE32);
    ret = adin2111_spi_write(dev, ADIN2111_MDIOACCn_START, (uint8_t *) &net_mdio_val, sizeof(net_mdio_val));
    if(ret != ADIN2111_ETH_SUCCESS) {
        goto end;
    }

    ret = adin2111_wait_mdio_ready(dev, ADIN2111_MDIOACCn_START + 1);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    /* Use the next MDIO register for the write operation. */
    mdio_cmd.MDIO_OP = ENUM_MAC_MDIOACC_N__MDIO_OP_MD_WR;
    mdio_cmd.MDIO_DATA = data;
    mdio_cmd.MDIO_TRDONE = 0;

    net_mdio_val = htonl(mdio_cmd.VALUE32);
    ret = adin2111_spi_write(dev, (ADIN2111_MDIOACCn_START + 1), (uint8_t *) &net_mdio_val, sizeof(net_mdio_val));
    if ( ret != ADIN2111_ETH_SUCCESS) {
        goto end;
    }

    ret = adin2111_wait_mdio_ready(dev, (ADIN2111_MDIOACCn_START + 1));
    if (ret != ADIN2111_ETH_SUCCESS) {
        goto end;
    }

end:
    return ret;
}

static adin2111_return_code_t adin2111_spi_phy_read(const struct device *dev, uint32_t reg_addr, uint16_t *reg_data, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    adin2111_mac_mdio_t mdio_cmd;

    /* Use the first MDIO register for the address operation.   */
    /* MDIO Speed defaults to 2.5MHz per CONFIG2.MSPEED.        */
    mdio_cmd.MDIO_DEVAD = DEVTYPE(reg_addr);
    mdio_cmd.MDIO_ST = ENUM_MAC_MDIOACC_N__MDIO_ST_CLAUSE45;
    mdio_cmd.MDIO_PRTAD = port_num;
    mdio_cmd.MDIO_OP = ENUM_MAC_MDIOACC_N__MDIO_OP_MD_ADDR;
    mdio_cmd.MDIO_DATA = REGADDR(reg_addr);
    mdio_cmd.MDIO_TRDONE = 0;

    uint32_t net_mdio_val = htonl(mdio_cmd.VALUE32);
    ret = adin2111_spi_write(dev, ADIN2111_MDIOACCn_START, (uint8_t *) &net_mdio_val, sizeof(net_mdio_val));
    if(ret != ADIN2111_ETH_SUCCESS)
    {
        goto end; 
    }

    /* Use the next MDIO register for the read operation. */
    mdio_cmd.MDIO_OP = ENUM_MAC_MDIOACC_N__MDIO_OP_MD_RD;
    mdio_cmd.MDIO_DATA = 0;
    mdio_cmd.MDIO_TRDONE = 0;

    net_mdio_val = htonl(mdio_cmd.VALUE32);
    ret = adin2111_spi_write(dev, (ADIN2111_MDIOACCn_START + 1), (uint8_t *) &net_mdio_val, sizeof(net_mdio_val));
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    ret = adin2111_wait_mdio_ready(dev, ADIN2111_MDIOACCn_START + 1);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    ret = adin2111_spi_read(dev, ADIN2111_MDIOACCn_START + 1, (uint8_t *) &mdio_cmd.VALUE32, sizeof(mdio_cmd.VALUE32));
    *reg_data = ntohl(mdio_cmd.VALUE32);

end:
    return ret;
}

static adin2111_return_code_t adin2111_write_frame(const struct device *dev, adin2111_mac_frame_t *frame) {
    const struct adin2111_config *cfg = dev->config;
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    adin2111_mac_spi_header_t spi_hdr;
    uint32_t byte_idx;
    uint8_t pad_size = 0;
    uint8_t cmd[ADIN2111_FRAME_HEADER_SIZE + ADIN2111_SPI_HEADER_SIZE];
    uint8_t pad[3] = {0};
    uint8_t spi_buf_count = 0;

    /* Frames need to be sized a multiple of 4 */
    if (frame->p_buf_desc->trx_size & 3) {
        pad_size = ADI_SPI_TX_FIFO_PADDING - (frame->p_buf_desc->trx_size & 3);
    }

    if (pad_size) {
        spi_buf_count = 3;
    } else {
        spi_buf_count = 2;
    }

    if ((frame->p_buf_desc->trx_size + pad_size) > NET_ETH_MAX_FRAME_SIZE)
    {
        ret = ADIN2111_ETH_PARAM_OUT_OF_RANGE;
        goto end;
    }

    byte_idx = 0;

    /* SPI header, this is placed in the first 2 bytes of the SPI transmite buffer */
    spi_hdr.CD = ADIN2111_MAC_SPI_TRANSACTION_CONTROL;
    spi_hdr.FD = ADIN2111_MAC_SPI_HALF_DUPLEX;
    spi_hdr.RW = ADIN2111_MAC_SPI_WRITE;
    spi_hdr.ADDR = ADIN2111_TX;
    cmd[byte_idx++] = spi_hdr.VALUE16 >> 8;
    cmd[byte_idx++] = spi_hdr.VALUE16 & 0xFF;

    /* TODO: allow for CRC */
#if defined(SPI_CRC_EN)
    // cmd[byteIdx++] = CrcBlock8(&spiTxBuf[0], ADI_SPI_HEADER_SIZE, 0, false);
#endif

    /* Append frame header */
    cmd[byte_idx++] = frame->header.VALUE16 >> 8;
    cmd[byte_idx++] = frame->header.VALUE16 & 0xFF;

    /* TODO: Determine if DMA is necessary based on size */

    const struct spi_buf tx_buf[3] = {
        {
            .buf = cmd,
            .len = byte_idx,
        },
        {
            .buf = frame->p_buf_desc->p_buf,
            .len = frame->p_buf_desc->trx_size,
        },
        {
            .buf = pad,
            .len = pad_size,
        },
    };

    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = spi_buf_count,
    };

    if(spi_write_dt(&cfg->spi, &tx) != 0) {
        ret = ADIN2111_ETH_SPI_ERROR;
    }

end:
    return ret;
}

static adin2111_return_code_t adin2111_receive_frame(const struct device *dev, uint8_t port) {
    struct adin2111_runtime *ctx = dev->data;
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint32_t rx_size = 0;
    uint32_t host_rx_size = 0;
    uint32_t reg_addr;

    reg_addr = ADIN2111_P1_RX_FSIZE;

    if (port)
    {
        reg_addr = ADIN2111_P2_RX_FSIZE;
    }

    if ((ret = adin2111_spi_read(dev, reg_addr, (uint8_t *) &rx_size, sizeof(rx_size))) != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }
    host_rx_size = ntohl(rx_size);

    /* Odd-size frames need an extra byte */
    if (host_rx_size & 1)
    {
        host_rx_size++;
    }

    reg_addr = ADIN2111_P1_RX;

    if (port)
    {
        reg_addr = ADIN2111_P2_RX;
    }

    ret = adin2111_spi_read(dev, reg_addr, ctx->buf, rx_size);

end:
  return ret;
}

/* ====================================================================================================
        ADIN2111 PHY 
   ==================================================================================================== */
static adin2111_return_code_t adin2111_phy_static_config(const struct device *dev, uint32_t model_num, uint32_t rev_num, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;

    if (rev_num == 0) {
        ret = adin2111_spi_phy_write(dev, 0x1E8C81, 0x0001, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }

        if (model_num == 10)
        {
            ret = adin2111_spi_phy_write(dev, 0x1E8C80, 0x0001, port_num);
        }
        else
        {
            ret = adin2111_spi_phy_write(dev, 0x1E8C80, 0x363, port_num);
        }
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }

        ret = adin2111_spi_phy_write(dev, 0x1E881F, 0x0000, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }

        ret = adin2111_spi_phy_write(dev, 0x018154, 0x00F9, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }

        ret = adin2111_spi_phy_write(dev, 0x1E8C40, 0x000B, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }

        ret = adin2111_spi_phy_write(dev, 0x018008, 0x0003, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x018009, 0x0008, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x018167, 0x2000, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x018168, 0x0008, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x01816B, 0x0400, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x0181BD, 0x2000, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x0181BE, 0x0008, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x0181C2, 0x0400, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x0181DB, 0x0400, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x0181E1, 0x0400, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x0181E7, 0x0400, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x0181EB, 0x0400, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin2111_spi_phy_write(dev, 0x018143, 0x0400, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }

        if (model_num == 10)
        {
            ret = adin2111_spi_phy_write(dev, 0x1EA400, 0x0001, port_num);
            if (ret != ADIN2111_ETH_SUCCESS)
            {
                goto end;
            }
            ret = adin2111_spi_phy_write(dev, 0x1EA407, 0x0001, port_num);
            if (ret != ADIN2111_ETH_SUCCESS)
            {
                goto end;
            }
        }
    }

end:
    return ret;
}

static adin2111_return_code_t adin2111_phy_get_sw_powerdown(const struct device *dev, bool *enable, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint16_t val = 0;

    ret = adin2111_spi_phy_read(dev, ADIN2111_CRSM_STAT, &val, port_num);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        ret = ADIN2111_ETH_COMM_ERROR;
        goto end;
    }

    *enable = ((val & BITM_CRSM_STAT_CRSM_SFT_PD_RDY) == (1 << BITP_CRSM_STAT_CRSM_SFT_PD_RDY));

end:
    return ret;
}

static adin2111_return_code_t adin2111_phy_set_sw_powerdown(const struct device *dev, bool enable, adin2111_port_t port_num) {
    adin2111_return_code_t result = ADIN2111_ETH_SUCCESS;
    uint16_t val;
    uint16_t bitval;
    bool swpd;
    int32_t iter = ADIN2111_PHY_SOFT_PD_ITER;

    bitval = (enable) ? 1: 0;
    val = bitval << BITP_CRSM_SFT_PD_CNTRL_CRSM_SFT_PD;
    result = adin2111_spi_phy_write(dev, ADIN2111_CRSM_SFT_PD_CNTRL, val, port_num);
    if (result != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    /* Wait with timeout for the PHY device to enter the desired state before returning. */
    do {
        result = adin2111_phy_get_sw_powerdown(dev, &swpd, port_num);
    } while ((val != (uint32_t) swpd) && (--iter));

    if (iter <= 0)
    {
        result = ADIN2111_ETH_READ_STATUS_TIMEOUT;
    }

end:
    return result;
}

static adin2111_return_code_t adin2111_phy_check_id(const struct device *dev, uint32_t *model_num, uint32_t *rev_num, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint16_t val;

    ret = adin2111_spi_phy_read(dev, ADIN2111_MMD1_DEV_ID1, &val, port_num);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }
    if (val != ADIN2111_PHY_DEVID1)
    {
        ret = ADIN2111_ETH_UNSUPPORTED_DEVICE;
        goto end;
    }

    ret = adin2111_spi_phy_read(dev, ADIN2111_MMD1_DEV_ID2, &val, port_num);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    /* Check if the value of MMD1_DEV_ID2.OUI matches expected value */
    if ((val & BITM_MMD1_DEV_ID2_MMD1_DEV_ID2_OUI) != (ADIN2111_PHY_DEVID2_OUI << BITP_MMD1_DEV_ID2_MMD1_DEV_ID2_OUI))
    {
        ret = ADIN2111_ETH_UNSUPPORTED_DEVICE;
    }

    *model_num = (uint32_t)((val & BITM_MMD1_DEV_ID2_MMD1_MODEL_NUM) >> BITP_MMD1_DEV_ID2_MMD1_MODEL_NUM);
    *rev_num = (uint32_t)((val & BITM_MMD1_DEV_ID2_MMD1_REV_NUM) >> BITP_MMD1_DEV_ID2_MMD1_REV_NUM);

end:
    return ret;
}

static adin2111_return_code_t adin2111_phy_an_enable(const struct device *dev, bool enable, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint16_t val;

    /* The only other bit in this register is AN_RESTART, need to write 0 to it */
    val = (enable? 1: 0) << BITP_AN_CONTROL_AN_EN;
    ret = adin2111_spi_phy_write(dev, ADIN2111_AN_CONTROL, val, port_num);

    return ret;
}

static adin2111_return_code_t adin2111_phy_reset(const struct device *dev, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint16_t val;

    val = (1 << BITP_CRSM_SFT_RST_CRSM_SFT_RST);
    ret = adin2111_spi_phy_write(dev, ADIN2111_CRSM_SFT_RST, val, port_num);

    if (ret == ADIN2111_ETH_SUCCESS)
    {
        ret = adin2111_phy_init(dev, port_num);
    }

    return ret;
}

static adin2111_return_code_t adin2111_phy_init(const struct device *dev, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint16_t val;
    uint32_t irqMask;
    bool flag;
    int32_t iter;
    uint32_t model_num;
    uint32_t rev_num;

    /* Checks the identity of the device based on reading of hardware ID registers */
    /* Ensures the device is supported by the driver, otherwise an error is reported. */
    ret = adin2111_phy_check_id(dev, &model_num, &rev_num, port_num);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    /* Go to software powerdown, this may already be achieved through pin strap options. */
    /* Note this is not using the driver function because we use a different timeout     */
    /* scheme to account for the powerup sequence of the system included in this step.   */
    val = 1 << BITP_CRSM_SFT_PD_CNTRL_CRSM_SFT_PD;
    ret = adin2111_spi_phy_write(dev, ADIN2111_CRSM_SFT_PD_CNTRL, val, port_num);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    /* iter = ADI_PHY_SYS_RDY_ITER; */
    iter = 1000;
    do
    {
        ret = adin2111_phy_get_sw_powerdown(dev, &flag, port_num);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
    } while (!flag && (--iter));

    /* Values of event enums are identical to respective interrupt masks */
    /* Hardware reset and hardware error interrupts are always enabled   */
    irqMask = ADIN2111_PHY_CRSM_HW_ERROR | BITM_CRSM_IRQ_MASK_CRSM_HRD_RST_IRQ_EN | BITM_PHY_SUBSYS_IRQ_MASK_LINK_STAT_CHNG_IRQ_EN;
    ret = adin2111_spi_phy_write(dev, ADIN2111_CRSM_IRQ_MASK, (irqMask & 0xFFFF), port_num);
    if (ret != ADIN2111_ETH_SUCCESS) 
    {
        goto end;
    }
    ret = adin2111_spi_phy_write(dev, ADIN2111_PHY_SUBSYS_IRQ_MASK, ((irqMask >> 16) & 0xFFFF), port_num);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    /* Read IRQ status bits to clear them before enabling IRQ. */
    /* Hardware errors could be asserted if for, we don't care about the contents */
    /* so we just discard the read values. */
    ret = adin2111_spi_phy_read(dev, ADIN2111_CRSM_IRQ_STATUS, &val, port_num);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }
    if (val & ADIN2111_PHY_CRSM_HW_ERROR)
    {
        ret = ADIN2111_ETH_HW_ERROR;
        goto end;
    }

    ret = adin2111_spi_phy_read(dev, ADIN2111_PHY_SUBSYS_IRQ_STATUS, &val, port_num);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    /* Static configuration: default settings that are different from hardware reset values */
    ret = adin2111_phy_static_config(dev, model_num, rev_num, port_num);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        ret = ADIN2111_ETH_PLACEHOLDER_ERROR;
        goto end;
    }

    /* Make sure auto-negotiation is enabled. */
    ret = adin2111_phy_an_enable(dev, true, port_num);

end:
    return ret;
}

/* ====================================================================================================
        Zephyr Integration functions
   ==================================================================================================== */
static adin2111_return_code_t adin2111_mac_reset(const struct device *dev, adin2111_eth_reset_type_t reset_type)
{
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint32_t key_1 = RST_MAC_ONLY_KEY1;
    uint32_t key_2 = RST_MAC_ONLY_KEY2;
    uint32_t retry_count;
    uint32_t reg_val = 0;
    uint32_t net_reg_val;

    /* Disable IRQ here to prevent RESETC from being cleared in the IRQ handler */
    /* Because a reset needs to be followed up by reconfiguration of the device */
    /* and a call to SyncConfig(), the IRQ will be enabled by the latter so it  */
    /* can be left in a disabled state by this reset function.                  */
    
    /* ADI_HAL_DISABLE_IRQ(hDevice->adinDevice); */

    /* When the reset is executed as part of the initialization routine, here   */
    /* is the first register access, and it may not be fully reset/powered up.  */
    /* To prevent a premature exit with an error, the reset is repeated several */
    /* times if the register write is not successful. This is particularly      */
    /* important when using OPEN Alliance protocol, because the comparison      */
    /* between the control header and the echoed control header will fail       */
    /* immediately if the device is sending invalid data (all 0s) to the host.  */
    retry_count = 0;

    do
    {
        switch (reset_type)
        {
            case ADIN2111_ETH_RESET_TYPE_MAC_ONLY:
                /* No checking of results, see comment above. */
                net_reg_val = htonl(key_1);
                ret = adin2111_spi_write(dev, ADIN2111_SOFT_RST, (uint8_t *) &net_reg_val, sizeof(net_reg_val));
                if (ret == ADIN2111_ETH_SUCCESS)
                {
                    net_reg_val = htonl(key_2);
                    ret = adin2111_spi_write(dev, ADIN2111_SOFT_RST, (uint8_t *) &net_reg_val, sizeof(net_reg_val));
                }

                break;
            case ADIN2111_ETH_RESET_TYPE_MAC_PHY:
                /* No checking of results, see comment above. */
                reg_val = 1 << BITP_MAC_RESET_SWRESET;
                net_reg_val = htonl(reg_val);
                ret = adin2111_spi_write(dev, ADIN2111_RESET, (uint8_t *) &net_reg_val, sizeof(net_reg_val));
                if (ret != ADIN2111_ETH_SUCCESS)
                {
                    goto end;
                }
                break;
            default:
                ret = ADIN2111_ETH_INVALID_PARAM;
                goto end;
        }
    } while ((ret != ADIN2111_ETH_SUCCESS) && (retry_count++ < ADIN2111_MAC_IF_UP_MAX_RETRIES));
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        ret = ADIN2111_ETH_SW_RESET_TIMEOUT;
        goto end;
    }
    if ((reset_type == ADIN2111_ETH_RESET_TYPE_MAC_ONLY) || (reset_type == ADIN2111_ETH_RESET_TYPE_MAC_PHY))
    {
        /* Wait for MAC reset to finish */
        ret = adin2111_wait_device_ready(dev);
        if (ret != ADIN2111_ETH_SUCCESS)
        {
            goto end;
        }
    }
    else
    {
        ret = ADIN2111_ETH_INVALID_PARAM;
    }

    /* TODO: Allow the MAC CRC to be configurable (Disabling currently) */
    ret = adin2111_mac_init(dev);

end:
    return ret;
}

static void adin2111_thread(const struct device *dev)
{
    struct adin2111_runtime *ctx = dev->data;
    const struct adin2111_config *config = dev->config;
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    adin2111_mac_status0_t status0;
    adin2111_mac_status1_t status1;
    uint32_t host_status0;
    uint32_t host_status1;
    uint16_t val;
    uint32_t reg_val;
    uint32_t host_val;

    while (true) {
        k_sem_take(&ctx->int_sem, K_FOREVER);

        while (gpio_pin_get_dt(&(config->interrupt))) {

            /* Read status0 register */
            ret = adin2111_spi_read(dev, ADIN2111_STATUS0, (uint8_t *) &status0.VALUE32, sizeof(status0.VALUE32));
            host_status0 = ntohl(status0.VALUE32);
            if (ret != ADIN2111_ETH_SUCCESS)
            {
                LOG_ERR("Error reading STATUS0 register");
            }

            /* Clear STATUS0 interrupts */
            ret = adin2111_spi_write(dev, ADIN2111_STATUS0, (uint8_t *) &status0.VALUE32, sizeof(status0.VALUE32));
            if (ret != ADIN2111_ETH_SUCCESS)
            {
                LOG_ERR("Error clearing STATUS0 register");
            }

            /* Read status1 register */
            ret = adin2111_spi_read(dev, ADIN2111_STATUS1, (uint8_t *) &status1.VALUE32, sizeof(status1.VALUE32));
            host_status1 = ntohl(status1.VALUE32);
            if (ret != ADIN2111_ETH_SUCCESS)
            {
                LOG_ERR("Error reading STATUS1 register");
            }

            /* TX Done */
            if (host_status1 & BITM_MAC_STATUS1_TX_RDY) {
                k_sem_give(&ctx->tx_sem);
                LOG_DBG("TX Done");

                ret = adin2111_spi_read(dev, ADIN2111_P1_TX_FRM_CNT, (uint8_t *) &reg_val, sizeof(reg_val));
                host_val = ntohl(reg_val);
                if (ret != ADIN2111_ETH_SUCCESS)
                {
                    LOG_ERR("Error reading P1_TX_FRM_CNT register");
                }
                LOG_DBG("P1 TX FRM CNT: %x", host_val);

                ret = adin2111_spi_read(dev, ADIN2111_P2_TX_BCAST_CNT, (uint8_t *) &reg_val, sizeof(reg_val));
                host_val = ntohl(reg_val);
                if (ret != ADIN2111_ETH_SUCCESS)
                {
                    LOG_ERR("Error reading ADIN2111_P2_TX_BCAST_CNT register");
                }
                LOG_DBG("P1 TX BCAST CNT: %x", host_val);

                ret = adin2111_spi_read(dev, ADIN2111_P2_TX_MCAST_CNT, (uint8_t *) &reg_val, sizeof(reg_val));
                host_val = ntohl(reg_val);
                if (ret != ADIN2111_ETH_SUCCESS)
                {
                    LOG_ERR("Error reading ADIN2111_P2_TX_MCAST_CNT register");
                }
                LOG_DBG("P1 TX MCAST CNT: %x", host_val);

                ret = adin2111_spi_read(dev, ADIN2111_P2_TX_UCAST_CNT, (uint8_t *) &reg_val, sizeof(reg_val));
                host_val = ntohl(reg_val);
                if (ret != ADIN2111_ETH_SUCCESS)
                {
                    LOG_ERR("Error reading ADIN2111_P2_TX_UCAST_CNT register");
                }
                LOG_DBG("P1 TX UCAST CNT: %x", host_val);
            }

            LOG_INF("STATUS0: %x", host_status0);
            LOG_INF("STATUS1: %x", host_status1);

            /* If PHYINT is asserted, read PHY status registers (cleared on read) */
            if (host_status0 & BITM_MAC_STATUS0_PHYINT) {
                //LOG_INF("PHY Interrupt on Port 1");
                ret = adin2111_spi_phy_read(dev, ADIN2111_CRSM_IRQ_STATUS, &val, ADIN2111_ETH_PORT_1);

                if (ret != ADIN2111_ETH_SUCCESS)
                {
                    LOG_ERR("Could not read CRSM IRQ Status reg");
                }
                //LOG_INF("Port 1 CRSM IRQ Status: %x", val);

                ret = adin2111_spi_phy_read(dev, ADIN2111_PHY_SUBSYS_IRQ_STATUS, &val, ADIN2111_ETH_PORT_1);
                if (ret != ADIN2111_ETH_SUCCESS)
                {
                    LOG_ERR("Could not read PHY SUBSYS IRQ Status reg");
                }
                //LOG_INF("Port 1 SUBSYS IRQ Status: %x", val);

            }
            if (host_status1 & BITM_MAC_STATUS1_P2_PHYINT) {
                //LOG_INF("PHY Interrupt on Port 2");
                ret = adin2111_spi_phy_read(dev, ADIN2111_CRSM_IRQ_STATUS, &val, ADIN2111_ETH_PORT_2);
                if (ret != ADIN2111_ETH_SUCCESS)
                {
                    LOG_ERR("Could not read CRSM IRQ Status reg");
                }
                //LOG_INF("Port 2 CRSM IRQ Status: %x", val);

                ret = adin2111_spi_phy_read(dev, ADIN2111_PHY_SUBSYS_IRQ_STATUS, &val, ADIN2111_ETH_PORT_2);
                if (ret != ADIN2111_ETH_SUCCESS)
                {
                    LOG_ERR("Could not read PHY SUBSYS IRQ Status reg");
                }
                //LOG_INF("Port 2 SUBSYS IRQ Status: %x", val);
            }

            if ((host_status0 & BITM_MAC_STATUS0_TTSCAA) || 
                (host_status0 & BITM_MAC_STATUS0_TTSCAB) ||
                (host_status0 & BITM_MAC_STATUS0_TTSCAC) || 
                (host_status1 & BITM_MAC_STATUS1_P2_TTSCAA) || 
                (host_status1 & BITM_MAC_STATUS1_P2_TTSCAB) ||
                (host_status1 & BITM_MAC_STATUS1_P2_TTSCAC)) {
                    LOG_DBG("TX Timestamp captured");
                /* TODO: Need callback for MAC timestamp ready */
            }

            if (host_status0 || host_status1) {
                /* TODO: Need call back for general interrupt */
            }

            /* Clear STATUS1 interrupts */
            ret = adin2111_spi_write(dev, ADIN2111_STATUS1, (uint8_t *) &status1.VALUE32, sizeof(status1.VALUE32));
            if (ret != ADIN2111_ETH_SUCCESS)
            {
                LOG_ERR("Error clearing STATUS0 register");
            }

            if (host_status1 & BITM_MAC_STATUS1_P1_RX_RDY ||
                host_status1 & BITM_MAC_STATUS1_P2_RX_RDY){
                /* RX data ready! */
                LOG_DBG("RX Data ready!");

            /* TODO: DISBALE IRQ HERE */
            // gpio_pin_interrupt_configure_dt(&config->interrupt,
            //                                 GPIO_INT_DISABLE);

                if (host_status1 & BITM_MAC_STATUS1_P2_RX_RDY) {
                    adin2111_receive_frame(dev, 1);
                } else {
                    adin2111_receive_frame(dev, 0);
                }

                /* Make sure the priority frames are being read first */
                if (host_status1 &  BITM_MAC_STATUS1_P1_RX_RDY_HI) {
                    adin2111_receive_frame(dev, 0);
                } else {
                    if (host_status1 & BITM_MAC_STATUS1_P2_RX_RDY_HI) {
                        adin2111_receive_frame(dev, 1);
                    } else {
                        if (host_status1 & BITM_MAC_STATUS1_P1_RX_RDY) {
                            adin2111_receive_frame(dev, 0);
                        } else {
                            adin2111_receive_frame(dev, 1);
                        }
                    }
                }
            }

        }
    }
}

static int adin2111_tx(const struct device *dev, struct net_pkt *pkt)
{
    struct adin2111_runtime *ctx = dev->data;
    uint16_t len = net_pkt_get_len(pkt);
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint32_t tx_fifo_avail;
    uint32_t host_tx_fifo_avail;
    uint32_t num_bytes;
    uint32_t net_num_bytes;
    adin2111_mac_frame_t p_frame;
    adin2111_eth_buf_desc_t p_buf_desc;

    if (net_pkt_read(pkt, ctx->buf, len)) {
        return -EIO;
    }

    p_buf_desc.p_buf = ctx->buf;
    p_buf_desc.trx_size = len + ADIN2111_FRAME_HEADER_SIZE;

    p_frame.p_buf_desc = &p_buf_desc;
    p_frame.header.VALUE16 = 0x0000;
    p_frame.header.EGRESS_CAPTURE = ADI_MAC_EGRESS_CAPTURE_A;

    /* TODO: hard code for now? */
    p_frame.header.PORT = 0x0;

    ret = adin2111_spi_read(dev, ADIN2111_TX_SPACE, (uint8_t *) &tx_fifo_avail, sizeof(tx_fifo_avail));
    host_tx_fifo_avail = ntohl(tx_fifo_avail);
    if (ret != ADIN2111_ETH_SUCCESS) {
        goto end;
    }

    num_bytes = p_buf_desc.trx_size;

    /* TX_SPACE contains the number of halfwords available in the TxFIFO, multiply by 2 to match the frame size in bytes */
    if (2* host_tx_fifo_avail < num_bytes + ADI_SPI_TX_FIFO_PADDING) {
        ret = ADIN2111_ETH_INSUFFICIENT_FIFO_SPACE;
        goto end;
    }

    net_num_bytes = htonl(num_bytes);
    ret = adin2111_spi_write(dev, ADIN2111_TX_FSIZE, (uint8_t *) &net_num_bytes, sizeof(net_num_bytes));
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    ret = adin2111_write_frame(dev, &p_frame);

    if (k_sem_take(&ctx->tx_sem, K_MSEC(10))) {
        return -EIO;
    }

end:
    return ret;
}

static adin2111_return_code_t adin2111_mac_init(const struct device *dev)
{
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint32_t imask0;
    uint32_t imask1;
    /* Eventually we need to enable at least the PHY_INT in addition to the   */
    /* existing interrupt sources.   */

    /* IMASK0 */
    /* All interrupt sources are unmasked, writing them individually for clarity */
    /* Exception is the PHY interrupt, it is too early to enable it here as this */
    /* is run before the PHY initialization.                                     */
    imask0 = 0xFFFFFFFF;
    imask0 &= ~(BITM_MAC_IMASK0_TXPEM |
                BITM_MAC_IMASK0_TXBOEM |
                BITM_MAC_IMASK0_TXBUEM |
                BITM_MAC_IMASK0_RXBOEM |
                BITM_MAC_IMASK0_LOFEM |
                BITM_MAC_IMASK0_HDREM |
                BITM_MAC_IMASK0_RESETCM |
                BITM_MAC_IMASK0_TXFCSEM |
                BITM_MAC_IMASK0_CDPEM);

    uint32_t net_imask0 = htonl(imask0);
    ret = adin2111_spi_write(dev, ADIN2111_IMASK0, (uint8_t *) &net_imask0, sizeof(net_imask0));

    if (ret != ADIN2111_ETH_SUCCESS) {
        ret = ADIN2111_ETH_COMM_ERROR;
        goto end;
    }

    /* IMASK1 */
    imask1 = 0xFFFFFFFF;
    imask1 &= ~(
                BITM_MAC_IMASK1_TX_RDY_MASK |
                BITM_MAC_IMASK1_P1_RX_RDY_MASK |
                BITM_MAC_IMASK1_P2_RX_RDY_MASK |
                BITM_MAC_IMASK1_P1_RX_IFG_ERR_MASK |
                BITM_MAC_IMASK1_SPI_ERR_MASK |
                BITM_MAC_IMASK1_RX_ECC_ERR_MASK |
                BITM_MAC_IMASK1_P2_TXFCSEM |
                BITM_MAC_IMASK1_TX_ECC_ERR_MASK);

    uint32_t net_imask1 = htonl(imask1);
    ret = adin2111_spi_write(dev, ADIN2111_IMASK1, (uint8_t *) &net_imask1, sizeof(net_imask1));

    if (ret != ADIN2111_ETH_SUCCESS) {
        ret = ADIN2111_ETH_COMM_ERROR;
        goto end;
    }

    if (ret != ADIN2111_ETH_SUCCESS) {
        ret = ADIN2111_ETH_COMM_ERROR;
        goto end;
    }

    if (ret != ADIN2111_ETH_SUCCESS)
    {
        goto end;
    }

    /* TODO: Need to modify CONFIG0 and CONFIG2 registers if we add CRC */
end:
    return ret;
}

static void adin2111_gpio_callback(const struct device *dev,
                struct gpio_callback *cb,
                uint32_t pins) {
    struct adin2111_runtime *ctx =
        CONTAINER_OF(cb, struct adin2111_runtime, gpio_cb);

    k_sem_give(&ctx->int_sem);
}

static void adin2111_iface_init(struct net_if *iface) {
    const struct device *dev = net_if_get_device(iface);
    struct adin2111_runtime *ctx = dev->data;

    net_if_set_link_addr(iface, ctx->mac_addr,
                 sizeof(ctx->mac_addr),
                 NET_LINK_ETHERNET);

    if (!ctx->iface) {
        ctx->iface = iface;
    }

    ethernet_init(iface);
}

static adin2111_return_code_t adin2111_memory_configure(const struct device *dev) 
{
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint32_t reg_val = 0;
    uint32_t net_reg_val = 0;
    uint8_t bit_pos = 0;

    /* Set to default FIFO sizes - Total must add up to 28K */
    reg_val |= (ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_8K);
    bit_pos += BITL_MAC_FIFO_SIZE_HTX_SIZE;
    reg_val |= (ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_12K << bit_pos);
    bit_pos += BITL_MAC_FIFO_SIZE_P1_RX_LO_SIZE;
    reg_val |= (ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_8K << bit_pos);

    net_reg_val = htonl(reg_val);
    ret = adin2111_spi_write(dev, ADIN2111_FIFO_SIZE, (uint8_t *) &net_reg_val, sizeof(net_reg_val));

    return ret;
}

static adin2111_return_code_t adin2111_wait_mdio_ready(const struct device *dev, uint16_t addr_offset)
{
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    adin2111_mac_mdio_t mdio_cmd;
    uint32_t retry_count = 0;
    bool mdio_rdy = false;
    uint32_t host_mdio_val;

    do {
        ret = adin2111_spi_read(dev, addr_offset, (uint8_t *) &mdio_cmd.VALUE32, sizeof(mdio_cmd.VALUE32));
        host_mdio_val = ntohl(mdio_cmd.VALUE32);

        if (ret != ADIN2111_ETH_SUCCESS)
        {
            break;
        }
        else if ((host_mdio_val & BITM_MAC_MDIOACC_N__MDIO_TRDONE) != 0)
        {
            mdio_rdy = true;
        }
    } while (((ret != ADIN2111_ETH_SUCCESS) || !mdio_rdy) && (retry_count++ < ADIN2111_MAC_MDIO_MAX_RETRIES));

    ret = ((ret == ADIN2111_ETH_SUCCESS) && mdio_rdy) ? ADIN2111_ETH_SUCCESS: ADIN2111_ETH_MDIO_TIMEOUT;
    return ret;
}

static enum ethernet_hw_caps adin2111_get_capabilities(const struct device *dev) {
    ARG_UNUSED(dev);
    return ETHERNET_LINK_10BASE_T;
}

static int adin2111_hw_start(const struct device *dev) {
    if(adin2111_phy_set_sw_powerdown(dev, false, ADIN2111_ETH_PORT_1) == ADIN2111_ETH_SUCCESS)
    {
        return adin2111_phy_set_sw_powerdown(dev, false, ADIN2111_ETH_PORT_2);
    }
}

static int adin2111_hw_stop(const struct device *dev) {
    if(adin2111_phy_set_sw_powerdown(dev, true, ADIN2111_ETH_PORT_1) == ADIN2111_ETH_SUCCESS)
    {
        return adin2111_phy_set_sw_powerdown(dev, true, ADIN2111_ETH_PORT_2);
    }
}

static adin2111_return_code_t adin2111_sync_config(const struct device *dev) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    const struct adin2111_config *config = dev->config;
    uint32_t reg_val;
    uint32_t net_reg_val;
    uint32_t host_val;

    ret = adin2111_spi_read(dev, ADIN2111_CONFIG0, (uint8_t *) &reg_val, sizeof(reg_val));
    host_val = ntohl(reg_val);
    if (ret == ADIN2111_ETH_SUCCESS) {
        host_val |= BITM_MAC_CONFIG0_SYNC;
        net_reg_val = htonl(host_val);
        ret = adin2111_spi_write(dev, ADIN2111_CONFIG0, (uint8_t *) &net_reg_val, sizeof(net_reg_val));
        
        /* CONFIG0.SYNC is set, we can now enable the IRQ. */
        gpio_pin_interrupt_configure_dt(&config->interrupt,
                                        GPIO_INT_EDGE_FALLING);
    }

    return ret;
}

static adin2111_return_code_t adin2111_enable_mac_loopback(const struct device *dev) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint32_t reg_val;
    uint32_t net_reg_val;
    uint32_t host_val;

    ret = adin2111_spi_read(dev, ADIN2111_P1_LOOP, (uint8_t *) &reg_val, sizeof(reg_val));
    host_val = ntohl(reg_val);
    if (ret == ADIN2111_ETH_SUCCESS) {
        host_val |= BITM_MAC_P1_LOOP_P1_LOOPBACK_EN;
        net_reg_val = htonl(host_val);
        ret = adin2111_spi_write(dev, ADIN2111_P1_LOOP, (uint8_t *) &net_reg_val, sizeof(net_reg_val));
    }
    ret = adin2111_spi_read(dev, ADIN2111_P1_LOOP, (uint8_t *) &reg_val, sizeof(reg_val));
    
    ret = adin2111_spi_read(dev, ADIN2111_P2_LOOP, (uint8_t *) &reg_val, sizeof(reg_val));
    host_val = ntohl(reg_val);
    if (ret == ADIN2111_ETH_SUCCESS) {
        host_val |= BITM_MAC_P2_LOOP_P2_LOOPBACK_EN;
        net_reg_val = htonl(host_val);
        ret = adin2111_spi_write(dev, ADIN2111_P2_LOOP, (uint8_t *) &net_reg_val, sizeof(net_reg_val));
    }
    ret = adin2111_spi_read(dev, ADIN2111_P2_LOOP, (uint8_t *) &reg_val, sizeof(reg_val));

    return ret;
}

static adin2111_return_code_t adin2111_enable_mac_if_loopback(const struct device *dev, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint16_t val;

    ret = adin2111_spi_phy_read(dev, ADIN2111_MAC_IF_LOOPBACK, &val, port_num);
    if (ret == ADIN2111_ETH_SUCCESS) {
        val |= BITM_MAC_IF_LOOPBACK_MAC_IF_LB_EN;
        ret = adin2111_spi_phy_write(dev, ADIN2111_MAC_IF_LOOPBACK, val, port_num);
    }
    ret = adin2111_spi_phy_read(dev, ADIN2111_MAC_IF_LOOPBACK, &val, port_num);

    return ret;
}

static adin2111_return_code_t adin2111_enable_pcs_loopback(const struct device *dev, adin2111_port_t port_num) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint16_t val;

    ret = adin2111_spi_phy_read(dev, ADIN2111_B10L_PCS_CNTRL, &val, port_num);
    if (ret == ADIN2111_ETH_SUCCESS) {
        val |= BITM_B10L_PCS_CNTRL_B10L_LB_PCS_EN;
        ret = adin2111_spi_phy_write(dev, ADIN2111_B10L_PCS_CNTRL, val, port_num);
    }
    ret = adin2111_spi_phy_read(dev, ADIN2111_B10L_PCS_CNTRL, &val, port_num);

    return ret;
}

static adin2111_return_code_t adin2111_set_macaddr(const struct device *dev) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    struct adin2111_runtime *ctx = dev->data;
    adi_mac_address_rule_t rule;
    uint32_t reg_val;
    uint32_t net_reg_val;

    rule.VALUE16 = 0;
    rule.TO_HOST = 1;
    rule.APPLY2PORT1 = 1;
    rule.APPLY2PORT2 = 1;

    adin2111_random_mac(ctx->mac_addr);

    reg_val = (rule.VALUE16 << 16) | (ctx->mac_addr[0] << 8) | ctx->mac_addr[1];
    net_reg_val = htonl(reg_val);

    ret = adin2111_spi_write(dev, ADIN2111_ADDR_FILT_UPRn_START, (uint8_t *) &net_reg_val, sizeof(net_reg_val));
    if (ret == ADIN2111_ETH_SUCCESS) {
        ret = adin2111_spi_write(dev, ADIN2111_ADDR_FILT_LWRn_START, (uint8_t *) &(ctx->mac_addr[2]), 4);
    }

    return ret;
}

static adin2111_return_code_t adin2111_wait_device_ready(const struct device *dev) {
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;
    uint32_t retry_count = 0;
    bool reset_done = false;
    uint32_t status;
    bool comm_ok = false;
    uint32_t phy_id = 0;
    uint32_t reset_val = BITM_MAC_STATUS0_RESETC;
    uint32_t net_reset_val;

    /* Poll PHYID register to establish the device has been brought up (powered up, out of reset). */
    while ((!comm_ok) && (retry_count++ < ADIN2111_MAC_INIT_MAX_RETRIES)) {
        ret = adin2111_spi_read(dev, ADIN2111_PHYID, (uint8_t *) &phy_id, sizeof(phy_id));
        uint32_t host_phy_id = ntohl(phy_id);

        if (ret == ADIN2111_ETH_SUCCESS && (host_phy_id == ADIN2111_MAC_PHYID)) {
            comm_ok = true;
        }
    }

    if (!comm_ok) {
        LOG_ERR("timeout waiting for correct ADDR_MAC_PHYID");
        ret = ADIN2111_ETH_COMM_TIMEOUT;
        goto end;
    }

    retry_count = 0;
    /* Now we can check RESETC without worrying about status0 comming back as all 0xF due to MAC-PHY still in reset. */
    while ((!reset_done) && (retry_count++ < ADIN2111_MAC_IF_UP_MAX_RETRIES)) {
        ret = adin2111_spi_read(dev, ADIN2111_STATUS0, (uint8_t *) &status, sizeof(status));
        uint32_t host_status = ntohl(status);

        if ((ret == ADIN2111_ETH_SUCCESS) && ((host_status & BITM_MAC_STATUS0_RESETC) == BITM_MAC_STATUS0_RESETC)) {
            reset_done = true;
            net_reset_val = htonl(reset_val);
            ret = adin2111_spi_write(dev, ADIN2111_STATUS0, (uint8_t *) &net_reset_val, sizeof(net_reset_val));
        }
    }
    if (!reset_done) {
        ret = ADIN2111_ETH_SW_RESET_TIMEOUT;
    }

end:
    return ret;
}

static struct ethernet_api adin2111_api_funcs = {
    .iface_api.init = adin2111_iface_init,
    .get_capabilities = adin2111_get_capabilities,
    // .set_config = adin2111_set_config,
    .start = adin2111_hw_start,
    .stop = adin2111_hw_stop,
    .send = adin2111_tx,
};

static void adin2111_random_mac(uint8_t *mac_addr) {
    uint32_t entropy = sys_rand32_get();

    mac_addr[0] = ADIN_MAC_ADDR_0;
    mac_addr[1] = ADIN_MAC_ADDR_1;
    mac_addr[2] = ADIN_MAC_ADDR_2;
    mac_addr[3] = (entropy >> 16) & 0xff;
    mac_addr[4] = (entropy >> 8) & 0xff;
    mac_addr[5] = (entropy >>  0) & 0xff;

    LOG_INF("Generated MAC Address: %x:%x:%x:%x:%x:%x", mac_addr[0], mac_addr[1], mac_addr[2],
                                                        mac_addr[3], mac_addr[4], mac_addr[5]);
}

static int adin2111_init(const struct device *dev) {
    const struct adin2111_config *config = dev->config;
    struct adin2111_runtime *ctx = dev->data;
    adin2111_return_code_t ret = ADIN2111_ETH_SUCCESS;

    if (!spi_is_ready(&config->spi)) {
        LOG_ERR("SPI master port %s not ready", config->spi.bus->name);
        return -EINVAL;
    }

    /* Interrupt pin */

    if (!device_is_ready(config->interrupt.port)) {
        LOG_ERR("GPIO port %s not ready", config->interrupt.port->name);
        return -EINVAL;
    }

    if (gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT)) {
        LOG_ERR("Unable to configure GPIO pin %u", config->interrupt.pin);
        return -EINVAL;
    }

    gpio_init_callback(&(ctx->gpio_cb), adin2111_gpio_callback,
               BIT(config->interrupt.pin));

    if (gpio_add_callback(config->interrupt.port, &(ctx->gpio_cb))) {
        return -EINVAL;
    }

    /* Reset pin */
    if (config->reset.port) {
        if (!device_is_ready(config->reset.port)) {
            LOG_ERR("GPIO port %s not ready", config->reset.port->name);
            return -EINVAL;
        }
        if (gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT)) {
            LOG_ERR("Unable to configure GPIO pin %u", config->reset.pin);
            return -EINVAL;
        }
        gpio_pin_set_dt(&config->reset, 0);
        k_usleep(500);
    }

    /* Resets and Initializes MAC*/
    if (adin2111_mac_reset(dev, ADIN2111_ETH_RESET_TYPE_MAC_PHY) != ADIN2111_ETH_SUCCESS) {
        while(1);
    }

    /* Configure forwarding of frames with unknown destination address */
    /* to the other port. This forwarding is done in hardware.         */
    /* Note that this setting will only take effect after the ports    */
    /* have been enabled (PHYs out of software powerdown).             */

    /* Configure the switch with cut through between ports enabled */
    /* and frames with unknown DAs forwarded to the other port.    */
    /* This is done in hardware and will take effect after the     */
    /* ports have been enabled (PHYs out of software powerdown).   */
    /* Note it's a read-modify-write because CONFIG2 maybe have    */
    /* been updated before (e.g. CRC_APPEND during MAC init.       */
    uint32_t reg_val;
    uint32_t host_val;
    uint32_t net_val;

    // ret = adin2111_spi_read(dev, ADIN2111_CONFIG2, (uint8_t *) &reg_val, sizeof(reg_val));
    // host_val = ntohl(reg_val);
    // if (ret != ADIN2111_ETH_SUCCESS)
    // {
    //     while(1);
    // }
    // host_val |= BITM_MAC_CONFIG2_P2_FWD_UNK2P2;
    // host_val |= BITM_MAC_CONFIG2_P2_FWD_UNK2P1;
    // host_val |= BITM_MAC_CONFIG2_PORT_CUT_THRU_EN;
    // net_val = htonl(host_val);
    // ret = adin2111_spi_write(dev, ADIN2111_CONFIG2, (uint8_t *) &net_val, sizeof(net_val));
    // if (ret != ADIN2111_ETH_SUCCESS)
    // {
    //     while(1);
    // }
    
    /* Init PHY1 */
    if (adin2111_phy_init(dev, ADIN2111_ETH_PORT_1) != ADIN2111_ETH_SUCCESS) {
        while(1);
    }

    /* Init PHY2 */
    if (adin2111_phy_init(dev, ADIN2111_ETH_PORT_2) != ADIN2111_ETH_SUCCESS) {
        while(1);
    }

    ret = adin2111_spi_read(dev, ADIN2111_CONFIG2, (uint8_t *) &reg_val, sizeof(reg_val));
    // host_val = ntohl(reg_val);
    // if (ret != ADIN2111_ETH_SUCCESS)
    // {
    //     while(1);
    // }

    /* Now enable the PHY interrupt sources in the MAC status registers */
    ret = adin2111_spi_read(dev, ADIN2111_IMASK0, (uint8_t *) &reg_val, sizeof(reg_val));
    host_val = ntohl(reg_val);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        while(1);
    }
    host_val &= ~BITM_MAC_IMASK0_PHYINTM;
    net_val = htonl(host_val);
    ret = adin2111_spi_write(dev, ADIN2111_IMASK0, (uint8_t *) &net_val, sizeof(net_val));
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        while(1);
    }

    ret = adin2111_spi_read(dev, ADIN2111_IMASK1, (uint8_t *) &reg_val, sizeof(reg_val));
    host_val = ntohl(reg_val);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        while(1);
    }
    host_val &= ~BITM_MAC_IMASK1_P2_PHYINT_MASK;
    net_val = htonl(host_val);
    ret = adin2111_spi_write(dev, ADIN2111_IMASK1, (uint8_t *) &net_val, sizeof(net_val));
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        while(1);
    }
    
    /* Set Destination MAC Address */
    if (adin2111_set_macaddr(dev) != ADIN2111_ETH_SUCCESS) {
        while(1);
    }

    /* Set rx/tx buffer sizes */
    // adin2111_memory_configure(dev);

    /* Debugging, MAC Loopback test for Channels 1 and 2 */
    // adin2111_enable_mac_loopback(dev);
    /* Debugging, MAC IF Loopback Test */
    // adin2111_enable_mac_if_loopback(dev);
    /* Debugging, PCS Loopback Test */
    // adin2111_enable_pcs_loopback(dev);

    /* Read CONFIG0 and CONFIG2 Registers */

    ret = adin2111_spi_read(dev, ADIN2111_CONFIG0, (uint8_t *) &reg_val, sizeof(reg_val));
    host_val = ntohl(reg_val);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        LOG_ERR("Error reading STATUS0 register");
    }
    LOG_INF("Config0 Reg Value: %x", host_val);


    ret = adin2111_spi_read(dev, ADIN2111_CONFIG2, (uint8_t *) &reg_val, sizeof(reg_val));
    host_val = ntohl(reg_val);
    if (ret != ADIN2111_ETH_SUCCESS)
    {
        LOG_ERR("Error reading STATUS0 register");
    }
    LOG_INF("Config2 Reg Value: %x", host_val);

    /* Prevents additonal changes to certain (FIX: Which?) registers before the device is started */
    adin2111_sync_config(dev);

    k_thread_create(&ctx->thread, ctx->thread_stack,
            CONFIG_ETH_ADIN2111_RX_THREAD_STACK_SIZE,
            (k_thread_entry_t)adin2111_thread,
            (void *)dev, NULL, NULL,
            K_PRIO_COOP(CONFIG_ETH_ADIN2111_RX_THREAD_PRIO),
            0, K_NO_WAIT);

    k_thread_name_set(&ctx->thread, "adin2111");

    LOG_INF("ADIN2111 Initialized");
    return ret;
}

static struct adin2111_runtime adin2111_0_runtime = {
    .generate_mac = adin2111_random_mac,
    .tx_sem = Z_SEM_INITIALIZER(adin2111_0_runtime.tx_sem,
                    1,  UINT_MAX),
    .int_sem  = Z_SEM_INITIALIZER(adin2111_0_runtime.int_sem,
                      0, UINT_MAX),
};

static const struct adin2111_config adin2111_0_config = {
    .spi = SPI_DT_SPEC_GET(DT_NODELABEL(spi_adin2111), SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),
    .interrupt = GPIO_DT_SPEC_INST_GET(0, int_gpios),
    .reset = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios, { 0 }),
    .timeout = CONFIG_ETH_ADIN2111_TIMEOUT,
};

ETH_NET_DEVICE_DT_INST_DEFINE(0,
            adin2111_init, NULL,
            &adin2111_0_runtime, &adin2111_0_config,
            CONFIG_ETH_INIT_PRIORITY, &adin2111_api_funcs, NET_ETH_MTU);