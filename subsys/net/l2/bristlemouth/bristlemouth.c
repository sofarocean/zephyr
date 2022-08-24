/*
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_l2_bristlemouth, CONFIG_NET_L2_BRISTLEMOUTH_LOG_LEVEL);

#include <zephyr/net/net_core.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/random/rand32.h>

#include <zephyr/net/bristlemouth.h>

#include "net_private.h"

#define NET_BUF_TIMEOUT K_MSEC(100)

static const struct net_bm_addr multicast_bm_addr __unused = {
	{ 0x33, 0x33, 0x00, 0x00, 0x00, 0x00 } };

static const struct net_bm_addr broadcast_bm_addr = {
	{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } };

const struct net_bm_addr *net_bm_broadcast_addr(void)
{
	return &broadcast_bm_addr;
}

void net_bm_ipv6_mcast_to_mac_addr(const struct in6_addr *ipv6_addr,
				    struct net_bm_addr *mac_addr)
{
	/* RFC 2464 7. Address Mapping -- Multicast
	 * "An IPv6 packet with a multicast destination address DST,
	 * consisting of the sixteen octets DST[1] through DST[16],
	 * is transmitted to the Ethernet multicast address whose
	 * first two octets are the value 3333 hexadecimal and whose
	 * last four octets are the last four octets of DST."
	 */
	mac_addr->addr[0] = mac_addr->addr[1] = 0x33;
	memcpy(mac_addr->addr + 2, &ipv6_addr->s6_addr[12], 4);
}

#define print_ll_addrs(pkt, type, len, src, dst)			   \
	if (CONFIG_NET_L2_BRISTLEMOUTH_LOG_LEVEL >= LOG_LEVEL_DBG) {	   \
		char out[sizeof("xx:xx:xx:xx:xx:xx")];			   \
									   \
		snprintk(out, sizeof(out), "%s",			   \
			 net_sprint_ll_addr((src)->addr,		   \
					    sizeof(struct net_bm_addr))); \
									   \
		NET_DBG("iface %p src %s dst %s type 0x%x len %zu",	   \
			net_pkt_iface(pkt), out,		   \
			net_sprint_ll_addr((dst)->addr,	   \
					    sizeof(struct net_bm_addr)), \
			type, (size_t)len);				   \
	}

static inline void bristlemouth_update_length(struct net_if *iface, struct net_pkt *pkt)
{
	uint16_t len;

	/* Let's check IP payload's length. If it's smaller than 46 bytes,
	 * i.e. smaller than minimal Bristlemouth frame size minus Bristlemouth
	 * header size,then Bristlemouth has padded so it fits in the minimal
	 * frame size of 60 bytes. In that case, we need to get rid of it.
	 */

	len = ntohs(NET_IPV6_HDR(pkt)->len) + NET_IPV6H_LEN;

	if (len < NET_BM_MINIMAL_FRAME_SIZE - sizeof(struct net_bm_hdr)) {
		struct net_buf *frag;

		for (frag = pkt->frags; frag; frag = frag->frags) {
			if (frag->len < len) {
				len -= frag->len;
			} else {
				frag->len = len;
				len = 0U;
			}
		}
	}
}

static void bristlemouth_update_rx_stats(struct net_if *iface,
				     struct net_pkt *pkt, size_t length)
{
	// TODO:
}


static enum net_verdict bristlemouth_recv(struct net_if *iface, struct net_pkt *pkt)
{
	struct bristlemouth_context *ctx = net_if_l2_data(iface);
	struct net_bm_hdr *hdr = NET_BM_HDR(pkt);
	uint8_t hdr_len = sizeof(struct net_bm_hdr);
	uint16_t type;
	struct net_linkaddr *lladdr;
	sa_family_t family;

	// TODO: unused
	(void)ctx;

	/* This expects that the Bristlemouth header is in the first net_buf
	 * fragment. This is a safe expectation here as it would not make
	 * any sense to split the Bristlemouth header to two net_buf's by the
	 * Bristlemouth driver.
	 */
	if (hdr == NULL || pkt->buffer->len < hdr_len) {
		LOG_ERR( "Dropping A" );
		goto drop;
	}

	type = ntohs(hdr->type);

	switch (type) {
	case NET_BM_PTYPE_IPV6:
		net_pkt_set_family(pkt, AF_INET6);
		family = AF_INET6;
		break;
	default:
		LOG_ERR("Unknown hdr type 0x%04x iface %p", type, iface);
		goto drop;
	}

	/* Set the pointers to ll src and dst addresses */
	lladdr = net_pkt_lladdr_src(pkt);
	lladdr->addr = hdr->src.addr;
	lladdr->len = sizeof(struct net_bm_addr);
	lladdr->type = NET_LINK_BRISTLEMOUTH;

	lladdr = net_pkt_lladdr_dst(pkt);
	lladdr->addr = hdr->dst.addr;
	lladdr->len = sizeof(struct net_bm_addr);
	lladdr->type = NET_LINK_BRISTLEMOUTH;

	// print_ll_addrs(pkt, type, net_pkt_get_len(pkt),
	// 			net_pkt_lladdr_src(pkt),
	// 			net_pkt_lladdr_dst(pkt));

	if (!net_bm_is_addr_broadcast((struct net_bm_addr *)lladdr->addr) &&
	    !net_bm_is_addr_multicast((struct net_bm_addr *)lladdr->addr) &&
	    !net_linkaddr_cmp(net_if_get_link_addr(iface), lladdr)) {
		// The bristlemouth frame is not for me as the link addresses are different.
		LOG_ERR("Dropping frame, not for me [%s]",
			net_sprint_ll_addr(
					   net_if_get_link_addr(iface)->addr,
					   sizeof(struct net_bm_addr)));
		goto drop;
	}

	net_buf_pull(pkt->frags, hdr_len);

	bristlemouth_update_rx_stats(iface, pkt, net_pkt_get_len(pkt) + hdr_len);

	bristlemouth_update_length(iface, pkt);

	//LOG_INF( "Accepting packet" );

	return NET_CONTINUE;
drop:
	// TODO: Add stats update
	// bm_stats_update_errors_rx(iface);

	return NET_DROP;
}

static bool bristlemouth_fill_in_dst_on_ipv6_mcast(struct net_pkt *pkt, struct net_bm_addr *dst)
{
	if (net_pkt_family(pkt) == AF_INET6 &&
	    net_ipv6_is_addr_mcast((struct in6_addr *)NET_IPV6_HDR(pkt)->dst)) {
		memcpy(dst, (uint8_t *)multicast_bm_addr.addr, sizeof(struct net_bm_addr) - 4);
		memcpy((uint8_t *)dst + 2,
		       NET_IPV6_HDR(pkt)->dst + 12,
		       sizeof(struct net_bm_addr) - 2);

		return true;
	}

	return false;
}

static struct net_buf *bristlemouth_fill_header(struct bristlemouth_context *ctx,
					    struct net_pkt *pkt,
					    uint32_t ptype)
{
	struct net_buf *hdr_frag;
	struct net_bm_hdr *hdr;

	// Allocate a fragment for the header (fixed size)
	hdr_frag = net_pkt_get_frag(pkt, NET_BUF_TIMEOUT);
	if (!hdr_frag) {
		return NULL;
	}

	// Point header struct at beginning of frag's data
	hdr = (struct net_bm_hdr *)(hdr_frag->data);

	// If the packet is a bound for a multicast address, update the destination mac address
	bristlemouth_fill_in_dst_on_ipv6_mcast(pkt, &hdr->dst);

	// Set source MAC address
	memcpy(&hdr->src, net_pkt_lladdr_src(pkt)->addr, sizeof(struct net_bm_addr));

	// Set ethertype
	hdr->type = ptype;

	// Add to network buffer
	net_buf_add(hdr_frag, sizeof(struct net_bm_hdr));

	print_ll_addrs(pkt, ntohs(hdr->type), hdr_frag->len, &hdr->src, &hdr->dst);

	// Insert fragment at beginning of packet
	net_pkt_frag_insert(pkt, hdr_frag);

	return hdr_frag;
}

static void bristlemouth_remove_l2_header(struct net_pkt *pkt)
{
	struct net_buf *buf;

	/* Remove the buffer added in bristlemouth_fill_header() */
	buf = pkt->buffer;
	pkt->buffer = buf->frags;
	buf->frags = NULL;

	net_pkt_frag_unref(buf);
}

static int bristlemouth_send( struct net_if *iface, struct net_pkt *pkt )
{
	const struct bristlemouth_api *api = net_if_get_device(iface)->api;
	struct bristlemouth_context *ctx = net_if_l2_data(iface);
	uint16_t ptype;
	int ret;

	if (!api) {
		ret = -ENOENT;
		goto error;
	}

	if (IS_ENABLED(CONFIG_NET_IPV6) && net_pkt_family(pkt) == AF_INET6) {
		ptype = htons(NET_BM_PTYPE_IPV6);
	} else if (IS_ENABLED(CONFIG_NET_SOCKETS_PACKET) &&
		   net_pkt_family(pkt) == AF_PACKET) {
		struct net_context *context = net_pkt_context(pkt);

		if (context && net_context_get_type(context) == SOCK_DGRAM) {
			struct sockaddr_ll *dst_addr;
			struct sockaddr_ll_ptr *src_addr;

			/* The destination address is set in remote for this
			 * socket type.
			 */
			dst_addr = (struct sockaddr_ll *)&context->remote;
			src_addr = (struct sockaddr_ll_ptr *)&context->local;

			net_pkt_lladdr_dst(pkt)->addr = dst_addr->sll_addr;
			net_pkt_lladdr_dst(pkt)->len =
						sizeof(struct net_bm_addr);
			net_pkt_lladdr_src(pkt)->addr = src_addr->sll_addr;
			net_pkt_lladdr_src(pkt)->len =
						sizeof(struct net_bm_addr);
			ptype = dst_addr->sll_protocol;
		} else {
			goto send;
		}
	}
	else {
		ret = -ENOTSUP;
		goto error;
	}

	/* If the ll dst addr has not been set before, let's assume
	 * temporarily it's a broadcast one. When filling the header,
	 * it might detect this should be multicast and act accordingly.
	 */
	if (!net_pkt_lladdr_dst(pkt)->addr) {
		net_pkt_lladdr_dst(pkt)->addr = (uint8_t *)broadcast_bm_addr.addr;
		net_pkt_lladdr_dst(pkt)->len = sizeof(struct net_bm_addr);
	}

	/* Then set the bristlemouth header.
	 */
	if (!bristlemouth_fill_header(ctx, pkt, ptype)) {
		ret = -ENOMEM;
		NET_DBG( "Error filling" );
		goto error;
	}

	net_pkt_cursor_init(pkt);

send:
	ret = net_l2_send(api->send, net_if_get_device(iface), iface, pkt);
	if (ret != 0) {
		// TODO: Update stats
		// bm_stats_update_errors_tx(iface);
		NET_DBG( "Error sending" );
		bristlemouth_remove_l2_header(pkt);
		goto error;
	}

	// TODO: Update stats
	// bristlemouth_update_tx_stats(iface, pkt);

	ret = net_pkt_get_len(pkt);
	bristlemouth_remove_l2_header(pkt);

	net_pkt_unref(pkt);
error:
	
	return ret;
}

static inline int bristlemouth_enable( struct net_if *iface, bool state )
{
	const struct bristlemouth_api *bm = 
		net_if_get_device( iface )->api;

	if (!bm) {
		return -ENOENT;
	}

	if (!state) {
        // TODO: Clear discovery tables?

		if (bm->stop) {
			bm->stop( net_if_get_device( iface ) );
		}
	} else {
		if (bm->start) {
			bm->start( net_if_get_device( iface ) );
		}
	}

	return 0;
}

enum net_l2_flags bristlemouth_flags( struct net_if *iface )
{
	struct bristlemouth_context *ctx = net_if_l2_data(iface);

	return ctx->bristlemouth_l2_flags;
}

NET_L2_INIT( BRISTLEMOUTH_L2, bristlemouth_recv, bristlemouth_send, bristlemouth_enable, bristlemouth_flags );

// TODO: Carrier logic can probably be removed.
// The current PHY HW impl does not expose carrier presence info to us. Driver will assume true
static void carrier_on_off(struct k_work *work)
{
	struct bristlemouth_context *ctx = CONTAINER_OF(work, struct bristlemouth_context,
						    carrier_work);
	bool bm_carrier_up;

	if (ctx->iface == NULL) {
		return;
	}

	bm_carrier_up = atomic_test_bit(&ctx->flags, BRISTLEMOUTH_CARRIER_UP);

	if (bm_carrier_up == ctx->is_net_carrier_up) {
		return;
	}

	ctx->is_net_carrier_up = bm_carrier_up;

	NET_DBG("Carrier %s for interface %p", bm_carrier_up ? "ON" : "OFF",
		ctx->iface);

	if (bm_carrier_up) {
		net_if_up(ctx->iface);
	} else {
		net_if_carrier_down(ctx->iface);
	}
}

void net_bm_carrier_on(struct net_if *iface)
{
	struct bristlemouth_context *ctx = net_if_l2_data(iface);

	if (!atomic_test_and_set_bit(&ctx->flags, BRISTLEMOUTH_CARRIER_UP)) {
		k_work_submit(&ctx->carrier_work);
	}
}

void net_eth_carrier_off(struct net_if *iface)
{
	struct bristlemouth_context *ctx = net_if_l2_data(iface);

	if (atomic_test_and_clear_bit(&ctx->flags, BRISTLEMOUTH_CARRIER_UP)) {
		k_work_submit(&ctx->carrier_work);
	}
}

static void setup_ipv6_link_local_addr(struct net_if *iface)
{
	struct net_if_addr *ifaddr;
	struct in6_addr addr;

	// Get the link address (MAC Address)
	struct net_linkaddr* link_addr = net_if_get_link_addr( iface );

	// Create an IPv6 Link-Local address using the standard technique for 6-byte MACs
	net_ipv6_addr_create_iid(&addr, link_addr);

	// Add as default link-local address
	ifaddr = net_if_ipv6_addr_add(iface, &addr, NET_ADDR_AUTOCONF, 0);
	if (!ifaddr) {
		NET_DBG("Cannot add %s address to interface %p", net_sprint_ipv6_addr(&addr), iface);
	}
}

static void setup_ipv6_realm_local_addr(struct net_if *iface)
{
	struct net_if_addr *ifaddr;
	struct in6_addr addr;

	// Get the link address (MAC Address)
	struct net_linkaddr* link_addr = net_if_get_link_addr( iface );

	// Create the ULA
	// TODO: Handle this using a more unique identifier
	// FC + Local bit = 0xFD
	// 40 Bits Global ID (0x00 0xAA 0xBB 0xCC 0xDD)
	// 16 bits Subnet ID (0x00 0x00)
	// 64 bits Interface ID (0xA000 <MAC ADDRESS>)
	net_ipv6_addr_create(&addr, 
		0xfd00, 0xAABB, 0xCCDD, 
		0x0000, 
		0xA000,
		link_addr->addr[0] << 8 | link_addr->addr[1], 
		link_addr->addr[2] << 8 | link_addr->addr[3],  
		link_addr->addr[4] << 8 | link_addr->addr[5]);

	ifaddr = net_if_ipv6_addr_add(iface, &addr, NET_ADDR_AUTOCONF, 0);
	if (!ifaddr) {
		NET_DBG("Cannot add %s address to interface %p", net_sprint_ipv6_addr(&addr), iface);
	}
}

void join_well_known_multicast_groups(struct net_if *iface)
{
	struct in6_addr ll_all_nodes_addr;
	struct in6_addr rl_all_nodes_addr;
	struct net_if_mcast_addr *maddr;

	net_ipv6_addr_create(&ll_all_nodes_addr, 0xff02, 0, 0, 0, 0, 0, 0, 0x0001);
	net_ipv6_addr_create(&rl_all_nodes_addr, 0xff03, 0, 0, 0, 0, 0, 0, 0x0001);
	
	// Add and join link-local
	maddr = net_if_ipv6_maddr_lookup(&ll_all_nodes_addr, &iface);
	if (maddr && net_if_ipv6_maddr_is_joined(maddr)) {
		// Already joined
	}

	if (!maddr) {
		maddr = net_if_ipv6_maddr_add(iface, &ll_all_nodes_addr);
		if (!maddr) {
			NET_ERR( "Failed to add LL multicast address" );
		}
	}

	net_if_ipv6_maddr_join(maddr);
	net_if_mcast_monitor(iface, &maddr->address, true);

	// Add and join realm
	maddr = net_if_ipv6_maddr_lookup(&rl_all_nodes_addr, &iface);
	if (maddr && net_if_ipv6_maddr_is_joined(maddr)) {
		// Already joined
	}

	if (!maddr) {
		maddr = net_if_ipv6_maddr_add(iface, &rl_all_nodes_addr);
		if (!maddr) {
			NET_ERR( "Failed to add RL multicast address" );
		}
	}

	net_if_ipv6_maddr_join(maddr);
	net_if_mcast_monitor(iface, &maddr->address, true);
}

void bristlemouth_init(struct net_if *iface)
{
	struct bristlemouth_context *ctx = net_if_l2_data(iface);

	NET_DBG("Initializing Bristlemouth L2 %p for iface %p", ctx, iface);

	// Set well-known unicast addresses
	setup_ipv6_link_local_addr( iface );
	setup_ipv6_realm_local_addr( iface );

	// Join well-known multicast groups
	join_well_known_multicast_groups( iface );

	ctx->bristlemouth_l2_flags = NET_L2_MULTICAST;
	ctx->iface = iface;
	k_work_init(&ctx->carrier_work, carrier_on_off);

	ctx->is_init = true;
}