/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME ieee802154_uart_pipe
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>

#include <kernel.h>
#include <arch/cpu.h>

#include <device.h>
#include <init.h>
#include <net/net_if.h>
#include <net/net_pkt.h>
#include <random/rand32.h>

#include <drivers/console/uart_pipe.h>
#include <net/ieee802154_radio.h>

#include "ieee802154_uart_pipe.h"
#include <net/openthread.h>

#define PAN_ID_OFFSET           3 /* Pan Id offset */
#define DEST_ADDR_OFFSET        5 /* Destination offset address*/
#define DEST_ADDR_TYPE_OFFSET   1 /* Destination address type */

#define DEST_ADDR_TYPE_MASK     0x0c /* Mask for destination address type */

#define DEST_ADDR_TYPE_SHORT    0x08 /* Short destination address type */
#define DEST_ADDR_TYPE_EXTENDED 0x0c /* Extended destination address type */

#define PAN_ID_SIZE           2    /* Size of Pan Id */
#define SHORT_ADDRESS_SIZE    2    /* Size of Short Mac Address */
#define EXTENDED_ADDRESS_SIZE 8    /* Size of Extended Mac Address */

/* Broadcast Short Address */
#define BROADCAST_ADDRESS    ((uint8_t [SHORT_ADDRESS_SIZE]) {0xff, 0xff})

static uint8_t dev_pan_id[PAN_ID_SIZE];             /* Device Pan Id */
static uint8_t dev_short_addr[SHORT_ADDRESS_SIZE];  /* Device Short Address */
static uint8_t dev_ext_addr[EXTENDED_ADDRESS_SIZE]; /* Device Extended Address */

/** Singleton device used in uart pipe callback */
static const struct device *upipe_dev;

static struct upipe_context upipe_context_data;

static uint16_t crc16_citt(uint16_t aFcs, uint8_t aByte)
{
    // CRC-16/CCITT, CRC-16/CCITT-TRUE, CRC-CCITT
    // width=16 poly=0x1021 init=0x0000 refin=true refout=true xorout=0x0000 check=0x2189 name="KERMIT"
    // http://reveng.sourceforge.net/crc-catalogue/16.htm#crc.cat.kermit
    static const uint16_t sFcsTable[256] = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5,
        0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52,
        0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3,
        0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9,
        0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
        0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f,
        0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862,
        0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb,
        0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948,
        0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226,
        0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497,
        0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704,
        0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb,
        0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
        0x3de3, 0x2c6a, 0x1ef1, 0x0f78};
    return (aFcs >> 8) ^ sFcsTable[(aFcs ^ aByte) & 0xff];
}

static void radioComputeCrc(uint8_t *aMessage, uint8_t* crc0, uint8_t* crc1, uint16_t aLength)
{
    uint16_t crc        = 0;
    uint16_t crc_offset = aLength - sizeof(uint16_t);

    for (uint16_t i = 0; i < crc_offset; i++)
    {
        crc = crc16_citt(crc, aMessage[i]);
    }

    *crc0 = crc & 0xff;
    *crc1 = crc >> 8;
}

void upipe_receive_failed(upipe_802154_rx_error_t error)
{
	const struct device *dev = net_if_get_device(upipe_context_data.iface);
	enum ieee802154_rx_fail_reason reason;

	switch (error) {
	case UPIPE_802154_RX_ERROR_INVALID_FRAME:
	case UPIPE_802154_RX_ERROR_DELAYED_TIMEOUT:
		reason = IEEE802154_RX_FAIL_NOT_RECEIVED;
		break;

	case UPIPE_802154_RX_ERROR_INVALID_FCS:
		reason = IEEE802154_RX_FAIL_INVALID_FCS;
		break;

	case UPIPE_802154_RX_ERROR_INVALID_DEST_ADDR:
		reason = IEEE802154_RX_FAIL_ADDR_FILTERED;
		break;

	default:
		reason = IEEE802154_RX_FAIL_OTHER;
		break;
	}

	// upipe_context_data.last_frame_ack_fpb = false;
	if (upipe_context_data.event_handler) {
		upipe_context_data.event_handler(dev, IEEE802154_EVENT_RX_FAILED, (void *)&reason);
	}
}

static uint8_t *upipe_rx(uint8_t *buf, size_t *off)
{
	struct net_pkt *pkt = NULL;
	struct upipe_context *upipe;

	if (!upipe_dev) {
		goto done;
	}

	upipe = upipe_dev->data;
	if (!upipe->rx && *buf == UART_PIPE_RADIO_15_4_FRAME_TYPE) {
		upipe->rx = true;
		goto done;
	}

	if (!upipe->rx_len) {
		if (*buf > 127) {
			goto flush;
		}

		upipe->rx_len = *buf;
		goto done;
	}

	upipe->rx_buf[upipe->rx_off++] = *buf;


	if (upipe->rx_len == upipe->rx_off) {
		LOG_DBG( "Got pkt of len %d", upipe->rx_len );
		struct net_buf *frag;

		pkt = net_pkt_rx_alloc(K_NO_WAIT);
		if (!pkt) {
			LOG_DBG("No pkt available");
			goto flush;
		}

		frag = net_pkt_get_frag(pkt, K_NO_WAIT);
		if (!frag) {
			LOG_DBG("No fragment available");
			goto out;
		}

		net_pkt_frag_insert(pkt, frag);

		memcpy(frag->data, upipe->rx_buf, upipe->rx_len);
		net_buf_add(frag, upipe->rx_len);

		if (ieee802154_radio_handle_ack(upipe->iface, pkt) == NET_OK) {
			
			/* Unsure of proper error code */
			upipe_receive_failed(UPIPE_802154_RX_ERROR_CATCHALL);
			
			LOG_DBG("ACK packet handled");
			goto out;
		}

		LOG_DBG("Caught a packet (%u)", upipe->rx_len);
		if (net_recv_data(upipe->iface, pkt) < 0) {

			/* Unsure of proper error code */
			upipe_receive_failed(UPIPE_802154_RX_ERROR_CATCHALL);

			LOG_DBG("Packet dropped by NET stack");
			goto out;
		}

		goto flush;
out:
		net_pkt_unref(pkt);
flush:
		upipe->rx = false;
		upipe->rx_len = 0U;
		upipe->rx_off = 0U;
	}
done:
	*off = 0;

	return buf;
}

static enum ieee802154_hw_caps upipe_get_capabilities(const struct device *dev)
{
	return IEEE802154_HW_FCS |
		IEEE802154_HW_2_4_GHZ |
		IEEE802154_HW_TX_RX_ACK |
		IEEE802154_HW_FILTER;
}

static int upipe_cca(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	if (upipe->stopped) {
		return -EIO;
	}

	return 0;
}

static int upipe_set_channel(const struct device *dev, uint16_t channel)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);

	return 0;
}

static int upipe_set_pan_id(const struct device *dev, uint16_t pan_id)
{
	uint8_t pan_id_le[2];

	ARG_UNUSED(dev);

	sys_put_le16(pan_id, pan_id_le);
	memcpy(dev_pan_id, pan_id_le, PAN_ID_SIZE);

	return 0;
}

static int upipe_set_short_addr(const struct device *dev, uint16_t short_addr)
{
	uint8_t short_addr_le[2];

	ARG_UNUSED(dev);

	sys_put_le16(short_addr, short_addr_le);
	memcpy(dev_short_addr, short_addr_le, SHORT_ADDRESS_SIZE);

	return 0;
}

static int upipe_set_ieee_addr(const struct device *dev,
			       const uint8_t *ieee_addr)
{
	ARG_UNUSED(dev);

	memcpy(dev_ext_addr, ieee_addr, EXTENDED_ADDRESS_SIZE);

	return 0;
}

static int upipe_filter(const struct device *dev,
			bool set,
			enum ieee802154_filter_type type,
			const struct ieee802154_filter *filter)
{
	LOG_DBG("Applying filter %u", type);

	if (!set) {
		return -ENOTSUP;
	}

	if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) {
		return upipe_set_ieee_addr(dev, filter->ieee_addr);
	} else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) {
		return upipe_set_short_addr(dev, filter->short_addr);
	} else if (type == IEEE802154_FILTER_TYPE_PAN_ID) {
		return upipe_set_pan_id(dev, filter->pan_id);
	}

	return -ENOTSUP;
}

static int upipe_set_txpower(const struct device *dev, int16_t dbm)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dbm);

	return 0;
}

static void upipe_tx_started(const struct device *dev,
			    struct net_pkt *pkt,
			    struct net_buf *frag)
{
	ARG_UNUSED(pkt);

	if (upipe_context_data.event_handler) {
		upipe_context_data.event_handler(dev, IEEE802154_EVENT_TX_STARTED,
					(void *)frag);
	}
}

static int upipe_tx(const struct device *dev,
		    enum ieee802154_tx_mode mode,
		    struct net_pkt *pkt,
		    struct net_buf *frag)
{
	struct upipe_context *upipe = dev->data;
	uint8_t *pkt_buf = frag->data;
	uint8_t len = frag->len;
	uint8_t i, data;

	if (mode != IEEE802154_TX_MODE_DIRECT) {
		NET_ERR("TX mode %d not supported", mode);
		return -ENOTSUP;
	}

	LOG_DBG( "Transmitting packet of length: %d", len );

	if (upipe->stopped) {
		return -EIO;
	}

	data = UART_PIPE_RADIO_15_4_FRAME_TYPE;
	uart_pipe_send(&data, 1);

	data = len + 2;
	uart_pipe_send(&data, 1);

	for (i = 0U; i < len; i++) {
		uart_pipe_send(pkt_buf+i, 1);
	}

	uint8_t crc0;
	uint8_t crc1;

	radioComputeCrc( pkt_buf, &crc0, &crc1, len );

	uart_pipe_send( &crc0, 1 );
	uart_pipe_send( &crc1, 1 );

	upipe_tx_started(dev, pkt, frag);

	return 0;
}

static int upipe_start(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	if (!upipe->stopped) {
		return -EALREADY;
	}

	upipe->stopped = false;

	return 0;
}

static int upipe_stop(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	if (upipe->stopped) {
		return -EALREADY;
	}

	upipe->stopped = true;

	return 0;
}

static int upipe_init(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	(void)memset(upipe, 0, sizeof(struct upipe_context));

	uart_pipe_register(upipe->uart_pipe_buf, 1, upipe_rx);

	upipe_stop(dev);

	return 0;
}

static inline uint8_t *get_mac(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	upipe->mac_addr[0] = 0x00;
	upipe->mac_addr[1] = 0x10;
	upipe->mac_addr[2] = 0x20;
	upipe->mac_addr[3] = 0x30;

#if defined(CONFIG_IEEE802154_UPIPE_RANDOM_MAC)
	UNALIGNED_PUT(sys_cpu_to_be32(sys_rand32_get()),
		      (uint32_t *) ((uint8_t *)upipe->mac_addr+4));
#else
	upipe->mac_addr[4] = CONFIG_IEEE802154_UPIPE_MAC4;
	upipe->mac_addr[5] = CONFIG_IEEE802154_UPIPE_MAC5;
	upipe->mac_addr[6] = CONFIG_IEEE802154_UPIPE_MAC6;
	upipe->mac_addr[7] = CONFIG_IEEE802154_UPIPE_MAC7;
#endif

	return upipe->mac_addr;
}

static void upipe_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct upipe_context *upipe = dev->data;
	uint8_t *mac = get_mac(dev);

	net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);

	upipe_dev = dev;
	upipe->iface = iface;

	ieee802154_init(iface);
}

static int upipe_configure(const struct device *dev,
			  enum ieee802154_config_type type,
			  const struct ieee802154_config *config)
{
	upipe_context_data.event_handler = config->event_handler;
	return 0;
}

static struct ieee802154_radio_api upipe_radio_api = {
	.iface_api.init		= upipe_iface_init,

	.get_capabilities	= upipe_get_capabilities,
	.cca			= upipe_cca,
	.set_channel		= upipe_set_channel,
	.filter			= upipe_filter,
	.set_txpower		= upipe_set_txpower,
	.tx			= upipe_tx,
	.start			= upipe_start,
	.stop			= upipe_stop,
	.configure		= upipe_configure,
};

#if defined(CONFIG_NET_L2_IEEE802154)
#define L2 IEEE802154_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(IEEE802154_L2)
#define MTU 125
#elif defined(CONFIG_NET_L2_OPENTHREAD)
#define L2 OPENTHREAD_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#define MTU 1280
#elif defined(CONFIG_NET_L2_CUSTOM_IEEE802154)
#define L2 CUSTOM_IEEE802154_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(CUSTOM_IEEE802154_L2)
#define MTU CONFIG_NET_L2_CUSTOM_IEEE802154_MTU
#endif

NET_DEVICE_INIT(upipe_15_4, CONFIG_IEEE802154_UPIPE_DRV_NAME,
		upipe_init, NULL, &upipe_context_data, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		&upipe_radio_api, L2,
		L2_CTX_TYPE, MTU);