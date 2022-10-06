#define DT_DRV_COMPAT	wiznet_w5500

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_adin1110, CONFIG_ETHERNET_LOG_LEVEL);

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
#include "eth_adin1110_priv.h"

static adin1110_return_code_t adin1110_spi_write(const struct device *dev, uint32_t addr, uint8_t *data, uint32_t len);
static adin1110_return_code_t adin1110_spi_read(const struct device *dev, uint32_t addr, uint8_t *data, uint32_t len);
static adin1110_return_code_t adin1110_spi_phy_write(const struct device *dev,  uint8_t hw_addr, uint32_t reg_addr, uint16_t data);
static adin1110_return_code_t adin1110_spi_phy_read(const struct device *dev, uint8_t hw_addr, uint32_t reg_addr, uint16_t *reg_data);

static adin1110_return_code_t adin1110_phy_static_config(const struct device *dev, uint32_t model_num, uint32_t rev_num);
static adin1110_return_code_t adin1110_phy_get_sw_powerdown(const struct device *dev, bool *enable);
static adin1110_return_code_t adin1110_phy_set_sw_powerdown(const struct device *dev, bool enable);
static adin1110_return_code_t adin1110_phy_check_id(const struct device *dev, uint32_t *model_num, uint32_t *rev_num);
static adin1110_return_code_t adin1110_phy_init(const struct device *dev);

static adin1110_return_code_t adin1110_mac_init(const struct device *dev);

static void adin1110_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void adin1110_iface_init(struct net_if *iface);
static adin1110_return_code_t adin1110_get_link_status(const struct device *dev,  adin1110_eth_link_status_t *linkStatus);
static adin1110_return_code_t adin1110_wait_mdio_ready(adi_mac_Device_t *hDevice, uint16_t addr_offset);
static enum ethernet_hw_caps adin1110_get_capabilities(const struct device *dev);
static int adin1110_hw_start(const struct device *dev);
static int adin1110_hw_stop(const struct device *dev);
static adin1110_return_code_t adin1110_sync_config(const struct device *dev);
static adin1110_return_code_t adin1110_set_macaddr(const struct device *dev);
static adin1110_return_code_t adin1110_wait_device_ready(const struct device *dev);

static void adin1110_random_mac(uint8_t *mac_addr);
static int adin1110_init(const struct device *dev);

/* ====================================================================================================
        ADIN1110 SPI Read/Write 
   ==================================================================================================== */
static adin1110_return_code_t adin1110_spi_write(const struct device *dev, uint32_t addr,
			   									 uint8_t *data, uint32_t len)
{
	const struct adin1110_config *cfg = dev->config;
	adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
	adin1110_mac_spi_header_t spi_hdr;
	uint8_t cmd[2];

    spi_hdr.CD = ADIN1110_MAC_SPI_TRANSACTION_CONTROL;
    spi_hdr.FD = ADIN1110_MAC_SPI_HALF_DUPLEX;
    spi_hdr.RW = ADIN1110_MAC_SPI_WRITE;
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

	if(spi_write_dt(&cfg->spi, &tx) != 0) {
		ret = ADIN1110_ETH_SPI_ERROR;
	}

	return ret;
}

static adin1110_return_code_t adin1110_spi_read(const struct device *dev, uint32_t addr,
			  uint8_t *data, uint32_t len)
{
	const struct adin1110_config *cfg = dev->config;
	adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
	uint8_t cmd[2];

    uint32_t tx_offset = ADIN1110_FRAME_HEADER_SIZE;
    adin1110_mac_spi_header_t spi_hdr;

	spi_hdr.CD = ADIN1110_MAC_SPI_TRANSACTION_CONTROL;
    spi_hdr.FD = ADIN1110_MAC_SPI_HALF_DUPLEX;
    spi_hdr.RW = ADIN1110_MAC_SPI_READ;
    spi_hdr.ADDR = (uint16_t) addr;
    spiTxBuf[0] = spi_hdr.VALUE16 >> 8;
    spiTxBuf[1] = spi_hdr.VALUE16 & 0xFF;

	// Bytes for registers + turnaround
    uint8_t total_offset = ADIN1110_FRAME_HEADER_SIZE + ADIN1110_TURNAROUND_SIZE;
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

	if (spi_transceive_dt(&cfg->spi, &tx, &rx) != 0) {
		ret = ADIN1110_ETH_SPI_ERROR;
	} else {
		memcpy(data, &tmp[total_offset], len);
	}

	return ret;
}

static adin1110_return_code_t adin1110_spi_phy_write(const struct device *dev, uint32_t reg_addr, uint16_t data)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    adin1110_mac_mdio_t mdio_cmd;

    /* Use the first MDIO register for the address operation. */
    /* MDIO Speed defaults to 2.5MHz per CONFIG2.MSPEED. */
    mdio_cmd.MDIO_DEVAD = DEVTYPE(reg_addr);
    mdio_cmd.MDIO_ST = ENUM_MAC_MDIOACC_N__MDIO_ST_CLAUSE45;
    mdio_cmd.MDIO_PRTAD = ADIN1110_PHY_ADDR;
    mdio_cmd.MDIO_OP = ENUM_MAC_MDIOACC_N__MDIO_OP_MD_ADDR;
    mdio_cmd.MDIO_DATA = REGADDR(reg_addr);
    mdio_cmd.MDIO_TRDONE = 0;

	ret = adin1110_spi_write(dev, ADIN1110_MDIOACCn_START, (uint8_t *) &mdioCmd.VALUE32, sizeof(mdioCmd.VALUE32));
    if(ret != ADIN1110_ETH_SUCCESS) {
        goto end;
    }

    /* Use the next MDIO register for the write operation. */
    mdio_cmd.MDIO_OP = ENUM_MAC_MDIOACC_N__MDIO_OP_MD_WR;
    mdio_cmd.MDIO_DATA = data;
    mdio_cmd.MDIO_TRDONE = 0;
	ret = adin1110_spi_write(dev, (ADIN1110_MDIOACCn_START + 1), (uint8_t *) &mdioCmd.VALUE32, sizeof(mdioCmd.VALUE32));
    if ( ret != ADIN1110_ETH_SUCCESS) {
        goto end;
    }

    ret = adin1110_wait_mdio_ready(dev, (ADIN1110_MDIOACCn_START + 1));
    if (ret != ADIN1110_ETH_SUCCESS) {
        goto end;
    }

end:
    return ret;
}

static adin1110_return_code_t adin1110_spi_phy_read(const struct device *dev, uint32_t reg_addr, uint16_t *reg_data)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    adin1110_mac_mdio_t mdio_cmd;

    /* Use the first MDIO register for the address operation.   */
    /* MDIO Speed defaults to 2.5MHz per CONFIG2.MSPEED.        */
    mdio_cmd.MDIO_DEVAD = DEVTYPE(reg_addr);
    mdio_cmd.MDIO_ST = ENUM_MAC_MDIOACC_N__MDIO_ST_CLAUSE45;
    mdio_cmd.MDIO_PRTAD = ADIN1110_PHY_ADDR;
    mdio_cmd.MDIO_OP = ENUM_MAC_MDIOACC_N__MDIO_OP_MD_ADDR;
    mdio_cmd.MDIO_DATA = REGADDR(reg_addr);
    mdio_cmd.MDIO_TRDONE = 0;

	ret = adin1110_spi_write(dev, ADIN1110_MDIOACCn_START, (uint8_t *) &mdioCmd.VALUE32, sizeof(mdioCmd.VALUE32));
    if(ret != ADIN1110_ETH_SUCCESS)
    {
        goto end; 
    }

    /* Use the next MDIO register for the read operation.       */
    mdioCmd.MDIO_OP = ENUM_MAC_MDIOACC_N__MDIO_OP_MD_RD;
    mdioCmd.MDIO_DATA = 0;
    mdioCmd.MDIO_TRDONE = 0;

	ret = adin1110_spi_write(dev, (ADIN1110_MDIOACCn_START + 1), (uint8_t *) &mdioCmd.VALUE32, sizeof(mdioCmd.VALUE32));
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

    ret = adin1110_wait_mdio_ready(dev, ADIN1110_MDIOACCn_START + 1);
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }
    ret = adin1110_spi_read(dev, ADIN1110_MDIOACCn_START + 1, (uint8_t *) &mdioCmd.VALUE32, sizeof(mdioCmd.VALUE32));
    *reg_data = mdioCmd.MDIO_DATA;

end:
    return ret;
}

/* ====================================================================================================
        ADIN1110 PHY 
   ==================================================================================================== */
   
static adin1110_return_code_t adin1110_phy_static_config(const struct device *dev, uint32_t model_num, uint32_t rev_num)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;

    if (rev_num == 0) {
        ret = adin1110_spi_phy_write(dev, 0x1E8C81, 0x0001);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }

        if (model_num == 10)
        {
            ret = adin1110_spi_phy_write(dev, 0x1E8C80, 0x0001);
        }
        else
        {
            ret = adin1110_spi_phy_write(dev, 0x1E8C80, 0x3636);
        }
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }

        ret = adin1110_spi_phy_write(dev, 0x1E881F, 0x0000);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }

        ret = adin1110_spi_phy_write(dev, 0x018154, 0x00F9);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }

        ret = adin1110_spi_phy_write(dev, 0x1E8C40, 0x000B);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }

        ret = adin1110_spi_phy_write(dev, 0x018008, 0x0003);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x018009, 0x0008);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x018167, 0x2000);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x018168, 0x0008);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x01816B, 0x0400);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x0181BD, 0x2000);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x0181BE, 0x0008);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x0181C2, 0x0400);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x0181DB, 0x0400);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x0181E1, 0x0400);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x0181E7, 0x0400);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x0181EB, 0x0400);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
        ret = adin1110_spi_phy_write(dev, 0x018143, 0x0400);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }

        if (model_num == 10)
        {
            ret = adin1110_spi_phy_write(dev, 0x1EA400, 0x0001);
            if (ret != ADIN1110_ETH_SUCCESS)
            {
                goto end;
            }
            ret = adin1110_spi_phy_write(dev, 0x1EA407, 0x0001);
            if (ret != ADIN1110_ETH_SUCCESS)
            {
                goto end;
            }
        }
    }

end:
    return ret;
}

static adin1110_return_code_t adin1110_phy_get_sw_powerdown(const struct device *dev, bool *enable)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    uint16_t val = 0;

    ret = adin1110_spi_phy_read(dev, ADIN1110_CRSM_STAT, val);
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        ret = ADIN1110_ETH_COMM_ERROR;
		goto end;
    }

    *enable = ((val & BITM_CRSM_STAT_CRSM_SFT_PD_RDY) == (1 << BITP_CRSM_STAT_CRSM_SFT_PD_RDY));

end:
    return ret;
}

static adin1110_return_code_t adin1110_phy_set_sw_powerdown(const struct device *dev, bool enable)
{
    adin1110_return_code_t result = ADIN1110_ETH_SUCCESS;
    uint16_t val;
    uint16_t bitval;
    bool swpd;
    int32_t iter = ADIN1110_PHY_SOFT_PD_ITER;

    bitval = (enable) ? 1: 0;
    val = bitval << BITP_CRSM_SFT_PD_CNTRL_CRSM_SFT_PD;
    result = adin1110_spi_phy_write(dev, ADIN1110_CRSM_SFT_PD_CNTRL, val);
    if (result != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

    /* Wait with timeout for the PHY device to enter the desired state before returning. */
    do
    {
        result = adin1110_phy_get_sw_powerdown(dev, &swpd);
    } while ((val != (uint32_t) swpd) && (--iter));

    if (iter <= 0)
    {
        result = ADIN1110_ETH_READ_STATUS_TIMEOUT;
    }

end:
    return result;
}

static adin1110_return_code_t adin1110_phy_check_id(const struct device *dev, uint32_t *model_num, uint32_t *rev_num)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    uint16_t val;

    ret = adin1110_spi_phy_read(dev, ADIN1110_MMD1_DEV_ID1, val);
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }
    if (val != ADIN1110_PHY_DEVID1)
    {
        ret = ADIN1110_ETH_UNSUPPORTED_DEVICE;
        goto end;
    }

    ret = adin1110_spi_phy_read(dev, ADIN1110_MMD1_DEV_ID2, val);
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

    /* Check if the value of MMD1_DEV_ID2.OUI matches expected value */
    if ((val & BITM_MMD1_DEV_ID2_MMD1_DEV_ID2_OUI) != (ADIN1110_PHY_DEVID2_OUI << BITP_MMD1_DEV_ID2_MMD1_DEV_ID2_OUI))
    {
        ret = ADIN1110_ETH_UNSUPPORTED_DEVICE;
    }

	*model_num = (uint32_t)((val & BITM_MMD1_DEV_ID2_MMD1_MODEL_NUM) >> BITP_MMD1_DEV_ID2_MMD1_MODEL_NUM);
    *rev_num = (uint32_t)((val & BITM_MMD1_DEV_ID2_MMD1_REV_NUM) >> BITP_MMD1_DEV_ID2_MMD1_REV_NUM);

end:
    return result;
}

static adin1110_return_code_t adin1110_phy_an_enable(const struct device *dev, bool enable)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    uint16_t val;

    if (hDevice->irqPending)
    {
        result = ADIN1110_ETH_IRQ_PENDING;
        goto end;
    }

    /* The only other bit in this register is AN_RESTART, need to write 0 to it */
    val = (enable? 1: 0) << BITP_AN_CONTROL_AN_EN;
    ret = adin1110_spi_phy_write(dev, ADIN1110_AN_CONTROL, val);

end:
    return ret;
}

static adin1110_return_code_t adin1110_phy_init(const struct device *dev)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    uint16_t val;
    uint32_t irqMask;
    bool flag;
    int32_t iter;
	uint32_t model_num;
    uint32_t rev_num;

    /* Checks the identity of the device based on reading of hardware ID registers */
    /* Ensures the device is supported by the driver, otherwise an error is reported. */
    ret = adin1110_phy_check_id(dev);
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

    /* Go to software powerdown, this may already be achieved through pin strap options. */
    /* Note this is not using the driver function because we use a different timeout     */
    /* scheme to account for the powerup sequence of the system included in this step.   */
    val = 1 << BITP_CRSM_SFT_PD_CNTRL_CRSM_SFT_PD;
    ret = adin1110_spi_phy_write(dev, ADIN1110_CRSM_SFT_PD_CNTRL, val);
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

    /* iter = ADI_PHY_SYS_RDY_ITER; */
	iter = 100;
    do
    {
        ret = adin1110_get_sw_powerdown(dev, &flag);
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            goto end;
        }
    } while (!flag && (--iter));

    /* Values of event enums are identical to respective interrupt masks */
    /* Hardware reset and hardware error interrupts are always enabled   */
    irqMask = ADIN1110_PHY_CRSM_HW_ERROR | BITM_CRSM_IRQ_MASK_CRSM_HRD_RST_IRQ_EN;
    ret = adin1110_spi_phy_write(dev, ADIN1110_CRSM_IRQ_MASK, (irqMask & 0xFFFF));
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }
    ret = adin1110_spi_phy_write(dev, ADIN1110_PHY_SUBSYS_IRQ_MASK, ((irqMask >> 16) & 0xFFFF));
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

    /* Read IRQ status bits to clear them before enabling IRQ. */
    /* Hardware errors could be asserted if for, we don't care about the contents */
    /* so we just discard the read values. */
    ret = adin1110_spi_phy_read(dev, ADIN1110_CRSM_IRQ_STATUS, &val);
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }
    if (val & ADIN1110_PHY_CRSM_HW_ERROR)
    {
        ret = ADIN1110_ETH_HW_ERROR;
        goto end;
    }

    ret = adin1110_spi_phy_read(dev, ADIN1110_PHY_SUBSYS_IRQ_STATUS, &val);
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

    /* Static configuration: default settings that are different from hardware reset values */
    ret = adin1110_phy_static_config(dev, model_num, rev_num);
    if (ret != ADIN1110_ETH_SUCCESS)
    {
        ret = ADIN1110_ETH_PLACEHOLDER_ERROR;
        goto end;
    }

    /* Make sure auto-negotiation is enabled. */
    ret = adin1110_phy_an_enable(dev, true);

end:
    return ret;
}

/* ====================================================================================================
        Zephyr Integration functions
   ==================================================================================================== */

static void adin1110_thread(const struct device *dev)
{
	uint8_t ir;
	struct adin1110_runtime *ctx = dev->data;
	const struct adin1110_config *config = dev->config;
	adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
	adin1110_mac_status0_t status0;
    adin1110_mac_status1_t status1;
	uint16_t val;

	while (true) {
		k_sem_take(&ctx->int_sem, K_FOREVER);

		while (gpio_pin_get_dt(&(config->interrupt))) {

			/* Read status0 register */
			ret = adin1110_spi_read(dev, ADIN1110_STATUS0, (uint8_t *) &status0.VALUE32, sizeof(status0.VALUE32));
			if (ret != ADIN1110_ETH_SUCCESS)
			{
				LOG_ERR("Error reading STATUS0 register");
			}

			/* Clear STATUS0 interrupts */
			ret = adin1110_spi_write(dev, ADIN1110_STATUS0, (uint8_t *) &status0.VALUE32, sizeof(status0.VALUE32));
			if (ret != ADIN1110_ETH_SUCCESS)
			{
				LOG_ERR("Error clearing STATUS0 register");
			}

			/* Read status1 register */
			ret = adin1110_spi_read(dev, ADIN1110_STATUS1, (uint8_t *) &status1.VALUE32, sizeof(status1.VALUE32));
			if (ret != ADIN1110_ETH_SUCCESS)
			{
				LOG_ERR("Error reading STATUS1 register");
			}

			/* If PHYINT is asserted, read PHY status registers (cleared on read) */
    		if (status0.VALUE32 & BITM_MAC_STATUS0_PHYINT) {
				ret = adin1110_spi_phy_read(dev, ADIN1110_CRSM_IRQ_STATUS, &val);
				if (ret != ADIN1110_ETH_SUCCESS)
				{
					LOG_ERR("Could not read CRSM IRQ Status reg");
				}

				/* Record state of CRSM_IRQ_STATUS */
				// hDevice->statusRegisters.p1Status &= 0xFFFF0000;
				// hDevice->statusRegisters.p1Status |= (val16 & 0x0000FFFF);
				// hDevice->statusRegisters.p1StatusMasked &= 0xFFFF0000;
				// hDevice->statusRegisters.p1StatusMasked |= (hDevice->statusRegisters.p1Status & hDevice->phyIrqMask & 0x0000FFFF);

				ret = adin1110_spi_phy_read(dev, ADIN1110_PHY_SUBSYS_IRQ_STATUS, &val);
				if (ret != ADIN1110_ETH_SUCCESS)
				{
					LOG_ERR("Could not read PHY SUBSYS IRQ Status reg");
				}

				/* Record state of PHY_SUBSYS_IRQ_STATUS */
				// hDevice->statusRegisters.p1Status &= 0x0000FFFF;
				// hDevice->statusRegisters.p1Status |= ((val16 << 16) & 0xFFFF0000);
				// hDevice->statusRegisters.p1StatusMasked &= 0x0000FFFF;
				// hDevice->statusRegisters.p1StatusMasked |= ((hDevice->statusRegisters.p1Status & hDevice->phyIrqMask & 0xFFFF0000) << 16);
			}
			if (status1.LINK_CHANGE) {
				/* TODO: Need callback for link change */
			}
			if (status0.TTSCAA | status0.TTSCAB | status0.TTSCAC) {
				/* TODO: Need callback for MAC timestamp ready */
			}
			if (status0Mask.VALUE32 || status1.VALUE32) {
				/* TODO: Need call back for general interrupt */
			}
			if (status1.P1_RX_RDY){
				/* RX data ready! */
			}

			/* Clear STATUS1 interrupts */
			ret = adin1110_spi_write(dev, ADIN1110_STATUS1, (uint8_t *) &status1.VALUE32, sizeof(status1.VALUE32));
			if (ret != ADIN1110_ETH_SUCCESS)
			{
				LOG_ERR("Error clearing STATUS0 register");
			}
		}
	}
}

static int adin1110_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct adin1110_runtime *ctx = dev->data;
	uint16_t len = net_pkt_get_len(pkt);
	int ret;

	if (net_pkt_read(pkt, ctx->buf, len)) {
		return -EIO;
	}



	w5500_command(dev, S0_CR_SEND);
	if (k_sem_take(&ctx->tx_sem, K_MSEC(10))) {
		return -EIO;
	}

	return 0;
}

static adin1110_return_code_t adin1110_mac_init(const struct device *dev)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    uint32_t reg_val;
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

    ret = adin1110_spi_write(dev, ADIN1110_IMASK0, imask0, sizeof(imask0));

	if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

    /* IMASK1 */
    imask1 = 0xFFFFFFFF;
    imask1 &= ~(BITM_MAC_IMASK1_P1_RX_IFG_ERR_MASK |
                BITM_MAC_IMASK1_SPI_ERR_MASK |
                BITM_MAC_IMASK1_RX_ECC_ERR_MASK |
                BITM_MAC_IMASK1_TX_ECC_ERR_MASK);

    ret = adin1110_spi_write(dev, ADIN1110_IMASK1, imask1, sizeof(imask1));

    if (ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

	/* TODO: Need to modify CONFIG0 and CONFIG2 registers if we add CRC */
end:
    return result;
}

static void adin1110_gpio_callback(const struct device *dev,
				struct gpio_callback *cb,
				uint32_t pins) {
	struct adin1110_runtime *ctx =
		CONTAINER_OF(cb, struct adin1110_runtime, gpio_cb);

	k_sem_give(&ctx->int_sem);
}

static void adin1110_iface_init(struct net_if *iface) {
	const struct device *dev = net_if_get_device(iface);
	struct adin1110_runtime *ctx = dev->data;

	net_if_set_link_addr(iface, ctx->mac_addr,
			     sizeof(ctx->mac_addr),
			     NET_LINK_ETHERNET);

	if (!ctx->iface) {
		ctx->iface = iface;
	}

	ethernet_init(iface);
}

static adin1110_return_code_t adin1110_memory_configure(const struct device *dev) 
{
	adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
	uint32_t reg_val = 0;
	uint8_t bit_pos = 0;

	/* Set to default FIFO sizes - Total must add up to 28K */
	reg_val |= (ENUM_MAC_FIFO_SIZE_HTX_SIZE_TXSIZE_8K);
	bit_pos += BITL_MAC_FIFO_SIZE_HTX_SIZE;
	reg_val |= (ENUM_MAC_FIFO_SIZE_P1_RX_LO_SIZE_RXSIZE_12K << bit_pos);
	bit_pos += BITL_MAC_FIFO_SIZE_P1_RX_LO_SIZE;
	reg_val |= (ENUM_MAC_FIFO_SIZE_P1_RX_HI_SIZE_RXSIZE_8K << bit_pos);

	ret = adin1110_spi_write(dev, ADIN1110_FIFO_SIZE, (uint8_t *) &reg_val, 2);

	return ret;
}

static adin1110_return_code_t adin1110_get_link_status(const struct device *dev,  adin1110_eth_link_status_t *linkStatus)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    uint32_t val;

    ret = adin1110_spi_read(dev, ADIN1110_STATUS1, (uint8_t *) &val, sizeof(val));
    if(ret != ADIN1110_ETH_SUCCESS)
    {
        goto end;
    }

    *linkStatus = (adin1110_eth_link_status_t)((val & BITM_MAC_STATUS1_P1_LINK_STATUS) >> BITP_MAC_STATUS1_P1_LINK_STATUS);

end:
    return ret;
}

static adin1110_return_code_t adin1110_wait_mdio_ready(const struct device *dev, uint16_t addr_offset)
{
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    adin1110_mac_mdio_t mdio_cmd;
    uint32_t retry_count = 0;
    bool mdio_rdy = false;

    do {
        ret = adin1110_spi_read(dev, addr_offset, (uint8_t *) &mdioCmd.VALUE32, sizeof(mdioCmd.VALUE32));
        if (ret != ADIN1110_ETH_SUCCESS)
        {
            break;
        }
        else if (mdioCmd.MDIO_TRDONE)
        {
            mdio_rdy = true;
        }
    } while (((ret != ADIN1110_ETH_SUCCESS) || !mdio_rdy) && (retry_count++ < ADIN1110_MAC_MDIO_MAX_RETRIES));

    ret = ((ret == ADIN1110_ETH_SUCCESS) && mdio_rdy) ? ADIN1110_ETH_SUCCESS: ADIN1110_ETH_MDIO_TIMEOUT;
    return ret;
}

static enum ethernet_hw_caps adin1110_get_capabilities(const struct device *dev) {
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE_T;
}

static int adin1110_hw_start(const struct device *dev) {
	return adin1110_phy_set_sw_powerdown(dev, false);
}

static int adin1110_hw_stop(const struct device *dev) {
	return adin1110_phy_set_sw_powerdown(dev, true);
}

static adin1110_return_code_t adin1110_sync_config(const struct device *dev) {
	adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
	uint32_t reg_val;
	struct adin1110_runtime *ctx = dev->data;

	ret = adin1110_spi_read(dev, ADIN1110_CONFIG0, (uint8_t *) &reg_val, sizeof(reg_val));
	if (ret == ADIN1110_ETH_SUCCESS) {
		reg_val |= BITM_MAC_CONFIG0_SYNC;
		ret = adin1110_spi_write(dev, ADIN1110_CONFIG0, (uint8_t *) &reg_val, sizeof(reg_val));
	}

	return ret;
}

static adin1110_return_code_t adin1110_set_macaddr(const struct device *dev) {
	adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
	struct adin1110_runtime *ctx = dev->data;

	/* override vendor bytes */
	memset(ctx->mac_addr, '\0', sizeof(ctx->mac_addr));
	ctx->mac_addr[0] = ADIN_MAC_ADDR_0;
	ctx->mac_addr[1] = ADIN_MAC_ADDR_1;
	ctx->mac_addr[2] = ADIN_MAC_ADDR_2;
	if (ctx->generate_mac) {
		ctx->generate_mac(ctx->mac_addr);
	}

	ret = adin1110_spi_write(dev, ADIN1110_ADDR_FILT_UPRn_START, (uint8_t *) (ctx->mac_addr), 2);
	if (ret == ADIN1110_ETH_SUCCESS) {
		ret = adin1110_spi_write(dev, ADIN1110_ADDR_FILT_LWRn_START, (uint8_t *) &(ctx->mac_addr[2]), 4);
	}

	return ret;
}

static adin1110_return_code_t adin1110_wait_device_ready(const struct device *dev) {
    adin1110_return_code_t ret = ADIN1110_ETH_SUCCESS;
    uint32_t retry_count = 0;
    bool reset_done = false;
    uint32_t status;
    bool comm_ok = false;
    uint32_t phy_id;

    /* Poll PHYID register to establish the device has been brought up (powered up, out of reset). */
    while ((!comm_ok) && (retry_count++ < ADIN1110_MAC_INIT_MAX_RETRIES)) {
        ret = adin1110_spi_read(dev, ADIN1110_PHYID, (uint8_t *) &phy_id, sizeof(phy_id));
        if ((ret == ADIN1110_ETH_SUCCESS) && (phy_id == BITM_MAC_CONFIG0_SYNC)) {
            comm_ok = true;
        }
    }

    if (!comm_ok) {
        LOG_ERR("timeout waiting for correct ADDR_MAC_PHYID");
        result = ADIN1110_ETH_COMM_TIMEOUT;
        goto end;
    }

	retry_count = 0;
    /* Now we can check RESETC without worrying about status0 comming back as all 0xF due to MAC-PHY still in reset. */
    while ((!reset_done) && (retry_count++ < ADIN1110_MAC_IF_UP_MAX_RETRIES)) {
        ret = adin1110_spi_read(dev, ADIN1110_STATUS0, (uint8_t *) &status, sizeof(status));
        if ((ret == ADIN1110_ETH_SUCCESS) && ((status & BITM_MAC_STATUS0_RESETC) == BITM_MAC_STATUS0_RESETC)) {
            reset_done = true;
            ret = adin1110_spi_write(dev, ADIN1110_STATUS0, BITM_MAC_STATUS0_RESETC);
        }
    }
    if (!reset_done) {
        ret = ADIN1110_ETH_SW_RESET_TIMEOUT;
    }

end:
    return ret;
}

static struct ethernet_api adin1110_api_funcs = {
	.iface_api.init = adin1110_iface_init,
	.get_capabilities = adin1110_get_capabilities,
	// .set_config = adin1110_set_config,
	.start = adin1110_hw_start,
	.stop = adin1110_hw_stop,
	// .send = adin1110_tx,
};

static void adin1110_random_mac(uint8_t *mac_addr) {
	gen_random_mac(mac_addr, ADIN_MAC_ADDR_0, ADIN_MAC_ADDR_1, ADIN_MAC_ADDR_2);
}

static int adin1110_init(const struct device *dev) {
	int err;
	uint8_t rtr[2];
	const struct adin1110_config *config = dev->config;
	struct adin1110_runtime *ctx = dev->data;

	if (!spi_is_ready(&config->spi)) {
		LOG_ERR("SPI master port %s not ready", config->spi.bus->name);
		return -EINVAL;
	}

	if (!device_is_ready(config->interrupt.port)) {
		LOG_ERR("GPIO port %s not ready", config->interrupt.port->name);
		return -EINVAL;
	}

	if (gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT)) {
		LOG_ERR("Unable to configure GPIO pin %u", config->interrupt.pin);
		return -EINVAL;
	}

	gpio_init_callback(&(ctx->gpio_cb), adin1110_gpio_callback,
			   BIT(config->interrupt.pin));

	if (gpio_add_callback(config->interrupt.port, &(ctx->gpio_cb))) {
		return -EINVAL;
	}

	gpio_pin_interrupt_configure_dt(&config->interrupt,
					GPIO_INT_EDGE_FALLING);

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

	/* Reset both MAC and PHY */
	adin1110_reset(dev);

	/* TODO: Allow the MAC CRC to be set */
	adin1110_mac_init(dev);

	/* Init PHY */
	adin1110_phy_init(dev);
	
	/* Set Destination MAC Address */
	adin1110_set_macaddr(dev);

	/* Set rx/tx buffer sizes */
	adin1110_memory_configure(dev);

	/* Prevents additonal changes to certain (TODO: Which?) registers before the device is started */
	adin1110_sync_config(dev);

	/* Enable device */
	adin1110_hw_start(dev);

	k_thread_create(&ctx->thread, ctx->thread_stack,
			CONFIG_ETH_ADIN1110_RX_THREAD_STACK_SIZE,
			(k_thread_entry_t)adin1110_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_ADIN1110_RX_THREAD_PRIO),
			0, K_NO_WAIT);

	LOG_INF("ADIN1110 Initialized");
	return 0;
}

static struct adin1110_runtime adin1110_0_runtime = {
	.generate_mac = adin1110_random_mac,
	.tx_sem = Z_SEM_INITIALIZER(adin1110_0_runtime.tx_sem,
					1,  UINT_MAX),
	.int_sem  = Z_SEM_INITIALIZER(adin1110_0_runtime.int_sem,
				      0, UINT_MAX),
};

static const struct adin1110_config adin1110_0_config = {
	.spi = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8), 0),
	.interrupt = GPIO_DT_SPEC_INST_GET(0, int_gpios),
	.reset = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios, { 0 }),
	.timeout = CONFIG_ETH_ADIN1110_TIMEOUT,
};

ETH_NET_DEVICE_DT_INST_DEFINE(0,
		    adin1110_init, NULL,
		    &adin1110_0_runtime, &adin1110_0_config,
		    CONFIG_ETH_INIT_PRIORITY, &adin1110_api_funcs, NET_ETH_MTU);