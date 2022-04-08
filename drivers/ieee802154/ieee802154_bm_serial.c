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

#include <drivers/console/bm_serial.h>
#include <net/ieee802154_radio.h>

#include "ieee802154_bm_serial.h"
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

static uint8_t tx_buf[MAX_BM_FRAME_SIZE];

static struct k_thread rx_thread_data;
K_THREAD_STACK_DEFINE(ieee802154_rx_stack, TASK_STACK_SIZE);

/** Singleton device used in uart pipe callback */
static const struct device *bm_upipe_dev;

static struct upipe_context bm_upipe_context_data;

void bm_upipe_receive_failed(upipe_802154_rx_error_t error)
{
	const struct device *dev = net_if_get_device(bm_upipe_context_data.iface);
	enum ieee802154_rx_fail_reason reason;

	switch (error) 
	{
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

	if (bm_upipe_context_data.event_handler) 
	{
		bm_upipe_context_data.event_handler(dev, IEEE802154_EVENT_RX_FAILED, (void *)&reason);
	}
}

static void bm_upipe_rx_thread(void)
{
	struct net_pkt *pkt = NULL;
	struct upipe_context *upipe;

	struct k_msgq* tx_queue = NULL;
	bm_msg_t msg;
	uint16_t frame_length;
	uint16_t payload_length;

	while (tx_queue != NULL)
	{
		tx_queue = bm_serial_get_rx_msgq_handler();
	} 

	upipe = bm_upipe_dev->data;

	while (1)
	{
		// Wait on bm_serial.c RX Thread to put message on queue
		k_msgq_get(tx_queue, &msg, K_FOREVER);
		frame_length = msg.frame_length;
		payload_length = frame_length - sizeof(bm_frame_header_t);
		uint8_t bm_version = ((bm_frame_header_t*) msg.frame_addr)->version;

		if (bm_version != BM_IEEE802154)
		{
			LOG_DBG("Incompatible version. Discarding Frame");
			goto done;
		}

		LOG_DBG( "Got pkt of len %d", frame_length );
		struct net_buf *frag;

		pkt = net_pkt_rx_alloc(K_NO_WAIT);
		if (!pkt) 
		{
			LOG_DBG("No pkt available");
			goto done;
		}

		frag = net_pkt_get_frag(pkt, K_NO_WAIT);
		if (!frag) 
		{
			LOG_DBG("No fragment available");
			goto unref;
		}

		net_pkt_frag_insert(pkt, frag);

		/* Memcpy payload into fragment */
		memcpy(frag->data, msg.frame_addr + sizeof(bm_frame_header_t), payload_length);
		net_buf_add(frag, payload_length);

		if (ieee802154_radio_handle_ack(upipe->iface, pkt) == NET_OK) 
		{
			LOG_DBG("ACK packet handled");
			goto unref;
		}

		if (net_recv_data(upipe->iface, pkt) < 0) 
		{
			/* Unsure of proper error code */
			bm_upipe_receive_failed(UPIPE_802154_RX_ERROR_CATCHALL);

			LOG_DBG("Packet dropped by NET stack");
			goto unref;
		}

		goto done;
unref:
		net_pkt_unref(pkt);
done:
		continue;
	}
}

static enum ieee802154_hw_caps bm_upipe_get_capabilities(const struct device *dev)
{
	return IEEE802154_HW_FCS |
		IEEE802154_HW_2_4_GHZ |
		IEEE802154_HW_TX_RX_ACK |
		IEEE802154_HW_FILTER;
}

static int bm_upipe_cca(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	if (upipe->stopped)
	{
		return -EIO;
	}

	return 0;
}

static int bm_upipe_set_channel(const struct device *dev, uint16_t channel)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);

	return 0;
}

static int bm_upipe_set_pan_id(const struct device *dev, uint16_t pan_id)
{
	uint8_t pan_id_le[2];

	ARG_UNUSED(dev);

	sys_put_le16(pan_id, pan_id_le);
	memcpy(dev_pan_id, pan_id_le, PAN_ID_SIZE);

	return 0;
}

static int bm_upipe_set_short_addr(const struct device *dev, uint16_t short_addr)
{
	uint8_t short_addr_le[2];

	ARG_UNUSED(dev);

	sys_put_le16(short_addr, short_addr_le);
	memcpy(dev_short_addr, short_addr_le, SHORT_ADDRESS_SIZE);

	return 0;
}

static int bm_upipe_set_ieee_addr(const struct device *dev,
			       const uint8_t *ieee_addr)
{
	ARG_UNUSED(dev);

	memcpy(dev_ext_addr, ieee_addr, EXTENDED_ADDRESS_SIZE);

	return 0;
}

static int bm_upipe_filter(const struct device *dev,
			bool set,
			enum ieee802154_filter_type type,
			const struct ieee802154_filter *filter)
{
	LOG_DBG("Applying filter %u", type);

	if (!set) 
	{
		return -ENOTSUP;
	}

	if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) 
	{
		return bm_upipe_set_ieee_addr(dev, filter->ieee_addr);
	} 
	else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) 
	{
		return bm_upipe_set_short_addr(dev, filter->short_addr);
	} 
	else if (type == IEEE802154_FILTER_TYPE_PAN_ID) 
	{
		return bm_upipe_set_pan_id(dev, filter->pan_id);
	}

	return -ENOTSUP;
}

static int bm_upipe_set_txpower(const struct device *dev, int16_t dbm)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dbm);

	return 0;
}

static void bm_upipe_tx_started(const struct device *dev,
			    struct net_pkt *pkt,
			    struct net_buf *frag)
{
	ARG_UNUSED(pkt);

	if (bm_upipe_context_data.event_handler) {
		bm_upipe_context_data.event_handler(dev, IEEE802154_EVENT_TX_STARTED,
					(void *)frag);
	}
}

static int bm_upipe_tx(const struct device *dev,
		    enum ieee802154_tx_mode mode,
		    struct net_pkt *pkt,
		    struct net_buf *frag)
{
	struct upipe_context *upipe = dev->data;
	uint8_t *pkt_buf = frag->data;
	uint8_t len = frag->len;
	int retval;

	if (mode != IEEE802154_TX_MODE_DIRECT) 
	{
		NET_ERR("TX mode %d not supported", mode);
		return -ENOTSUP;
	}

	LOG_DBG( "Transmitting packet of length: %d", len );

	if (upipe->stopped) 
	{
		return -EIO;
	}

	bm_frame_header_t bm_frm_hdr = { .version= BM_V0, .payload_type= BM_IEEE802154, .payload_length= len};
	memcpy(tx_buf, &bm_frm_hdr, sizeof(bm_frame_header_t));
	memcpy(&tx_buf[sizeof(bm_frame_header_t)], pkt_buf, bm_frm_hdr.payload_length);

	bm_frame_t *bm_frm = (bm_frame_t *)tx_buf;

	retval = bm_serial_frm_put(bm_frm);
	if (!retval)
	{
		bm_upipe_tx_started(dev, pkt, frag);
	}
	else
	{
		LOG_ERR( "TX MessageQueue is full, dropping message!");
	}

	return retval;
}

static int bm_upipe_start(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	if (!upipe->stopped) 
	{
		return -EALREADY;
	}

	upipe->stopped = false;

	return 0;
}

static int bm_upipe_stop(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	if (upipe->stopped) 
	{
		return -EALREADY;
	}

	upipe->stopped = true;

	return 0;
}

static int bm_upipe_init(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	(void)memset(upipe, 0, sizeof(struct upipe_context));

	bm_serial_init();

	k_thread_create(&rx_thread_data, ieee802154_rx_stack,
		K_THREAD_STACK_SIZEOF(ieee802154_rx_stack),
		(k_thread_entry_t)bm_upipe_rx_thread,
		NULL, NULL, NULL, K_PRIO_COOP(5), 0, K_NO_WAIT);

	bm_upipe_stop(dev);

	return 0;
}

static inline uint8_t *get_mac(const struct device *dev)
{
	struct upipe_context *upipe = dev->data;

	upipe->mac_addr[0] = 0x00;
	upipe->mac_addr[1] = 0x10;
	upipe->mac_addr[2] = 0x20;
	upipe->mac_addr[3] = 0x30;

#if defined(CONFIG_IEEE802154_BM_UPIPE_RANDOM_MAC)
	UNALIGNED_PUT(sys_cpu_to_be32(sys_rand32_get()),
		      (uint32_t *) ((uint8_t *)upipe->mac_addr+4));
#else
	upipe->mac_addr[4] = CONFIG_IEEE802154_BM_UPIPE_MAC4;
	upipe->mac_addr[5] = CONFIG_IEEE802154_BM_UPIPE_MAC5;
	upipe->mac_addr[6] = CONFIG_IEEE802154_BM_UPIPE_MAC6;
	upipe->mac_addr[7] = CONFIG_IEEE802154_BM_UPIPE_MAC7;
#endif

	return upipe->mac_addr;
}

static void bm_upipe_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct upipe_context *upipe = dev->data;
	uint8_t *mac = get_mac(dev);

	net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);

	bm_upipe_dev = dev;
	upipe->iface = iface;

	ieee802154_init(iface);
}

static int bm_upipe_configure(const struct device *dev,
			  enum ieee802154_config_type type,
			  const struct ieee802154_config *config)
{
	bm_upipe_context_data.event_handler = config->event_handler;
	return 0;
}

static struct ieee802154_radio_api bm_upipe_radio_api = 
{
	.iface_api.init			= bm_upipe_iface_init,
	.get_capabilities		= bm_upipe_get_capabilities,
	.cca					= bm_upipe_cca,
	.set_channel			= bm_upipe_set_channel,
	.filter					= bm_upipe_filter,
	.set_txpower			= bm_upipe_set_txpower,
	.tx						= bm_upipe_tx,
	.start					= bm_upipe_start,
	.stop					= bm_upipe_stop,
	.configure				= bm_upipe_configure,
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

NET_DEVICE_INIT(upipe_15_4, CONFIG_IEEE802154_BM_UPIPE_DRV_NAME,
		bm_upipe_init, NULL, &bm_upipe_context_data, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		&bm_upipe_radio_api, L2,
		L2_CTX_TYPE, MTU);