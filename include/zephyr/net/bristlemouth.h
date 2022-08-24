/*
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_BRISTLEMOUTH_H_
#define ZEPHYR_INCLUDE_NET_BRISTLEMOUTH_H_

#include <kernel.h>
#include <stdbool.h>

#include <zephyr/types.h>

#include <sys/util.h>
#include <sys/atomic.h>

#include <net/net_if.h>
#include <net/net_ip.h>
#include <net/net_pkt.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bristlemouth L2/driver support functions
 * @defgroup bristlemouth Bristlemouth L2/driver Support Functions
 * @ingroup networking
 * @{
 */

/** @cond INTERNAL_HIDDEN */

struct net_bm_addr {
	uint8_t addr[6];
};

#define NET_BM_HDR(pkt) ((struct net_bm_hdr *)net_pkt_data(pkt))

#define NET_BM_PTYPE_IP		    0x0800
#define NET_BM_PTYPE_IPV6		0x86dd
#define NET_BM_PTYPE_ALL        0x0003 /* from linux/if_ether.h */

#if !defined(BM_P_ALL)
#define BM_P_ALL	NET_BM_PTYPE_ALL
#endif
#if !defined(BM_P_IP)
#define BM_P_IP	    NET_BM_PTYPE_IP
#endif
#if !defined(BM_P_IPV6)
#define BM_P_IPV6		NET_BM_PTYPE_IPV6
#endif

#define NET_BM_MINIMAL_FRAME_SIZE	60
#define NET_BM_MTU					1500

#define _NET_BM_MAX_FRAME_SIZE	(NET_BM_MTU + sizeof(struct net_bm_hdr))
#define _NET_BM_MAX_HDR_SIZE    (sizeof(struct net_bm_hdr))
#define NET_BM_MAX_FRAME_SIZE 	(_NET_BM_MAX_FRAME_SIZE)
#define NET_BM_MAX_HDR_SIZE 	(_NET_BM_MAX_HDR_SIZE)

/** @endcond */

/** BRISTLEMOUTH hardware capabilities */
enum bristlemouth_hw_caps {
	/** Enabling/disabling auto negotiation supported */
	BRISTLEMOUTH_AUTO_NEGOTIATION_SET	= BIT(0),

	/** Changing duplex (half/full) supported */
	BRISTLEMOUTH_DUPLEX_SET		= BIT(1),
};

/** @cond INTERNAL_HIDDEN */

enum bristlemouth_config_type {
	BRISTLEMOUTH_CONFIG_TYPE_AUTO_NEG,
	BRISTLEMOUTH_CONFIG_TYPE_DUPLEX,
	BRISTLEMOUTH_CONFIG_TYPE_MAC_ADDRESS,
	BRISTLEMOUTH_CONFIG_TYPE_PRIORITY_QUEUES_NUM,
	BRISTLEMOUTH_CONFIG_TYPE_PORTS_NUM,
};

/** @endcond */

/** @cond INTERNAL_HIDDEN */
struct bristlemouth_config {
	union {
		bool auto_negotiation;
		bool full_duplex;

		struct net_bm_addr mac_address;

		int priority_queues_num;
		int ports_num;
	};
};
/** @endcond */

struct bristlemouth_api {
	/**
	 * The net_if_api must be placed in first position in this
	 * struct so that we are compatible with network interface API.
	 */
	struct net_if_api iface_api;

    /** Start the device */
	int (*start)(const struct device *dev);

	/** Stop the device */
	int (*stop)(const struct device *dev);

	/** Get the device capabilities */
	enum bristlemouth_hw_caps (*get_capabilities)(const struct device *dev);

	/** Set specific hardware configuration */
	int (*set_config)(const struct device *dev,
			  enum bristlemouth_config_type type,
			  const struct bristlemouth_config *config);

	/** Get hardware specific configuration */
	int (*get_config)(const struct device *dev,
			  enum bristlemouth_config_type type,
			  struct bristlemouth_config *config);

	/** Send a network packet */
	int (*send)(const struct device *dev, struct net_pkt *pkt);
};

/* Make sure that the network interface API is properly setup inside
 * Bristlemouth API struct (it is the first one).
 */
BUILD_ASSERT(offsetof(struct bristlemouth_api, iface_api) == 0);

/** @cond INTERNAL_HIDDEN */
struct net_bm_hdr {
	struct net_bm_addr dst;
	struct net_bm_addr src;
	uint16_t type;
	uint8_t src_port;
	uint16_t crc;
} __packed;

/** @endcond */

enum bristlemouth_flags {
	BRISTLEMOUTH_CARRIER_UP,
};

struct bristlemouth_context {
	/** Flags representing bristlemouth state, which are accessed from multiple threads. */
	atomic_t flags;

	/** Carrier ON/OFF handler worker. This is used to create
	 * network interface UP/DOWN event when bristlemouth L2 driver
	 * notices carrier ON/OFF situation. We must not create another
	 * network management event from inside management handler thus
	 * we use worker thread to trigger the UP/DOWN event.
	 */
	struct k_work carrier_work;

	/** Network interface. */
	struct net_if *iface;

	/**
	 * This tells what L2 features does ethernet support.
	 */
	enum net_l2_flags bristlemouth_l2_flags;

	/** Is network carrier up */
	bool is_net_carrier_up : 1;

	/** Is this context already initialized */
	bool is_init : 1;
};

/**
 * @brief Initialize Bristlemouth L2 stack for a given interface
 *
 * @param iface A valid pointer to a network interface
 */
void bristlemouth_init(struct net_if *iface);

#define BRISTLEMOUTH_L2_CTX_TYPE	struct bristlemouth_context

static inline bool net_bm_is_addr_broadcast(struct net_bm_addr *addr)
{
	if (addr->addr[0] == 0xff &&
	    addr->addr[1] == 0xff &&
	    addr->addr[2] == 0xff &&
	    addr->addr[3] == 0xff &&
	    addr->addr[4] == 0xff &&
	    addr->addr[5] == 0xff) {
		return true;
	}

	return false;
}

static inline bool net_bm_is_addr_unspecified(struct net_bm_addr *addr)
{
	if (addr->addr[0] == 0x00 &&
	    addr->addr[1] == 0x00 &&
	    addr->addr[2] == 0x00 &&
	    addr->addr[3] == 0x00 &&
	    addr->addr[4] == 0x00 &&
	    addr->addr[5] == 0x00) {
		return true;
	}

	return false;
}

static inline bool net_bm_is_addr_multicast(struct net_bm_addr *addr)
{
#if defined(CONFIG_NET_IPV6)
	if (addr->addr[0] == 0x33 &&
	    addr->addr[1] == 0x33) {
		return true;
	}
#endif

	return false;
}

const struct net_bm_addr *net_bm_broadcast_addr(void);

/**
 * @brief Convert IPv6 multicast address to Bristlemouth address.
 *
 * @param ipv6_addr IPv6 multicast address
 * @param mac_addr Output buffer for Bristlemouth address
 */
void net_bm_ipv6_mcast_to_mac_addr(const struct in6_addr *ipv6_addr,
				    struct net_bm_addr *bm_addr);

/**
 * @brief Return bristlemouth device hardware capability information.
 *
 * @param iface Network interface
 *
 * @return Hardware capabilities
 */
static inline
enum bristlemouth_hw_caps net_bm_get_hw_capabilities(struct net_if *iface)
{
	const struct bristlemouth_api *bm =
		(struct bristlemouth_api *)net_if_get_device(iface)->api;

	if (!bm->get_capabilities) {
		return (enum bristlemouth_hw_caps)0;
	}

	return bm->get_capabilities(net_if_get_device(iface));
}

#define Z_BM_NET_DEVICE_INIT(node_id, dev_name, drv_name, init_fn,	\
			      pm_action_cb, data, cfg, prio, api, mtu)	\
	Z_NET_DEVICE_INIT(node_id, dev_name, drv_name, init_fn,		\
			  pm_action_cb, data, cfg, prio, api,		\
			  BRISTLEMOUTH_L2, NET_L2_GET_CTX_TYPE(BRISTLEMOUTH_L2),\
			  mtu)

/**
 * @def BM_NET_DEVICE_INIT
 *
 * @brief Create a Bristlemouth network interface and bind it to network device.
 *
 * @param dev_name Network device name.
 * @param drv_name The name this instance of the driver exposes to
 * the system.
 * @param init_fn Address to the init function of the driver.
 * @param pm_action_cb Pointer to PM action callback.
 * Can be NULL if not implemented.
 * @param data Pointer to the device's private data.
 * @param cfg The address to the structure containing the
 * configuration information for this instance of the driver.
 * @param prio The initialization level at which configuration occurs.
 * @param api Provides an initial pointer to the API function struct
 * used by the driver. Can be NULL.
 * @param mtu Maximum transfer unit in bytes for this network interface.
 */
#define BM_NET_DEVICE_INIT(dev_name, drv_name, init_fn, pm_action_cb,	\
			    data, cfg, prio, api, mtu)			\
	Z_BM_NET_DEVICE_INIT(DT_INVALID_NODE, dev_name, drv_name,	\
			      init_fn, pm_action_cb, data, cfg, prio,	\
			      api, mtu)

/**
 * @brief Inform bristlemouth L2 driver that bristlemouth carrier is detected.
 * This happens when cable is connected.
 *
 * @param iface Network interface
 */
void net_bm_carrier_on(struct net_if *iface);

/**
 * @brief Inform bristlemouth L2 driver that bristlemouth carrier was lost.
 * This happens when cable is disconnected.
 *
 * @param iface Network interface
 */
void net_bm_carrier_off(struct net_if *iface);

#ifdef __cplusplus
}
#endif


/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_NET_DUMMY_H_ */
