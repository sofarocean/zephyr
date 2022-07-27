/*
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_l2_bristlemouth, CONFIG_NET_L2_BRISTLEMOUTH_LOG_LEVEL);

#include <net/net_core.h>
#include <net/net_l2.h>
#include <net/net_if.h>
#include <net/net_pkt.h>

#include <net/bristlemouth.h>

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
		goto drop;
	}

	type = ntohs(hdr->type);

	switch (type) {
	case NET_BM_PTYPE_IPV6:
		net_pkt_set_family(pkt, AF_INET6);
		family = AF_INET6;
		break;
	default:
		NET_DBG("Unknown hdr type 0x%04x iface %p", type, iface);
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
		NET_DBG("Dropping frame, not for me [%s]",
			net_sprint_ll_addr(
					   net_if_get_link_addr(iface)->addr,
					   sizeof(struct net_bm_addr)));
		goto drop;
	}

	net_buf_pull(pkt->frags, hdr_len);

	bristlemouth_update_rx_stats(iface, pkt, net_pkt_get_len(pkt) + hdr_len);

	bristlemouth_update_length(iface, pkt);

	return NET_CONTINUE;
drop:
	// TODO: Add stats update
	// bm_stats_update_errors_rx(iface);

	return NET_DROP;
}

static struct net_buf *bristlemouth_fill_header(struct bristlemouth_context *ctx,
					    struct net_pkt *pkt,
					    uint32_t ptype)
{
	struct net_buf *hdr_frag;
	struct net_bm_hdr *hdr;

	hdr_frag = net_pkt_get_frag(pkt, NET_BUF_TIMEOUT);
	if (!hdr_frag) {
		return NULL;
	}

	hdr = (struct net_bm_hdr *)(hdr_frag->data);

	memcpy(&hdr->src, net_pkt_lladdr_src(pkt)->addr, sizeof(struct net_bm_addr));

	hdr->type = ptype;
	net_buf_add(hdr_frag, sizeof(struct net_bm_hdr));

	// print_ll_addrs(pkt, ntohs(hdr->type), hdr_frag->len, &hdr->src, &hdr->dst);

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
	} else {
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
		goto error;
	}

	net_pkt_cursor_init(pkt);

	ret = net_l2_send(api->send, net_if_get_device(iface), iface, pkt);
	if (ret != 0) {
		// TODO: Update stats
		// bm_stats_update_errors_tx(iface);
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

void bristlemouth_init(struct net_if *iface)
{
	struct bristlemouth_context *ctx = net_if_l2_data(iface);

	NET_DBG("Initializing Bristlemouth L2 %p for iface %p", ctx, iface);

	ctx->bristlemouth_l2_flags = NET_L2_MULTICAST;
	ctx->iface = iface;
	k_work_init(&ctx->carrier_work, carrier_on_off);

	ctx->is_init = true;
}
