/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Sofar Ocean Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_BRISTLEMOUTH_BM_RX_FRAME_H_
#define ZEPHYR_INCLUDE_DRIVERS_BRISTLEMOUTH_BM_RX_FRAME_H_

#include <zephyr/sys/mpsc_pbuf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Message types
#define BM_RX_FRAME_TYPE_DEFAULT 0

// Occupies 3-bits
#define BM_RX_FRAME_GENERIC_HDR \
	MPSC_PBUF_HDR;\
	uint32_t type:1

// Generic Message Descriptor
struct bm_rx_frame_generic_hdr {
	BM_RX_FRAME_GENERIC_HDR;
};

// ============================================================
// Default message description

// Occupies a total of 32 bits (first item of any packet)
struct bm_rx_frame_desc {
	BM_RX_FRAME_GENERIC_HDR; // :3
    uint32_t phy_idx            :4;     // Which PHY device index the frame came in on
	uint32_t data_len           :16;    // Length of encoded frame (preamble is not included)
	uint32_t reserved           :9;
};

struct bm_rx_frame_hdr {
	struct bm_rx_frame_desc desc;

    // Placeholder - Can put other info here if needed (timestamps, counters, etc)
};

struct bm_rx_frame {
	struct bm_rx_frame_hdr hdr;

    // Data described as C99 flexible array member
    uint8_t data[];
};

// Union of frame types
union bm_rx_frame_generic {
	union mpsc_pbuf_generic         buf;
	struct bm_rx_frame_generic_hdr  generic;
	struct bm_rx_frame              frame;
};

// Macros for accessing wlen based on packet length
#define BM_RX_FRAME_ALIGNMENT (sizeof(uint32_t))

#define BM_RX_FRAME_ALIGN_OFFSET \
	offsetof(struct bm_rx_frame, data)

#define BM_RX_FRAME_LEN(data_len) \
	(offsetof(struct bm_rx_frame, data) + (data_len))

#define BM_RX_FRAME_LEN_ALIGNED_WLEN(data_len) \
	ceiling_fraction(ROUND_UP(BM_RX_FRAME_LEN(data_len), BM_RX_FRAME_ALIGNMENT), sizeof(uint32_t))

// ============================================================
// API

static inline struct bm_rx_frame* bm_rx_frame_alloc( struct mpsc_pbuf_buffer* pbuf, uint16_t dlen )
{
    size_t wlen = BM_RX_FRAME_LEN_ALIGNED_WLEN( dlen );

    return (struct bm_rx_frame*)mpsc_pbuf_alloc( pbuf, wlen, K_NO_WAIT );
}

static inline void bm_rx_frame_write_desc( struct bm_rx_frame *frame, uint8_t phy_idx, uint16_t dlen )
{
    // Create descriptor
    struct bm_rx_frame_desc desc = {
        .valid = 0,
        .busy = 0,
        .type = BM_RX_FRAME_TYPE_DEFAULT,
        .phy_idx = phy_idx,
        .data_len = dlen,
        .reserved = 0
    };

    frame->hdr.desc = desc;
}

static inline void bm_rx_frame_commit( struct mpsc_pbuf_buffer* pbuf, struct bm_rx_frame *frame )
{
    // Commit
    union bm_rx_frame_generic* m = (union bm_rx_frame_generic*)frame;
    mpsc_pbuf_commit(pbuf, &m->buf);
}

static inline uint8_t *bm_rx_frame_get_data( struct bm_rx_frame *frame, size_t *len_out )
{
	*len_out = frame->hdr.desc.data_len;

	return frame->data;
}

static inline int bm_rx_frame_finalize( struct mpsc_pbuf_buffer* pbuf, struct bm_rx_frame *frame, const struct bm_rx_frame_desc desc, const void *data )
{
    if (!frame || !pbuf || !data ) 
    {
        return -1;
    }

    // Copy data
    uint8_t* d = frame->data;
    memcpy(d, data, desc.data_len);

    // Copy header info
    frame->hdr.desc = desc;

    // Commit
    union bm_rx_frame_generic* m = (union bm_rx_frame_generic*)frame;
    mpsc_pbuf_commit(pbuf, &m->buf);

    return 0;
}

static inline int bm_rx_frame_queue( struct mpsc_pbuf_buffer* pbuf, uint8_t phy_idx, const uint8_t* data, uint16_t len )
{
    // TODO: Determine 32-bit word length of frame based on input packet length in bytes
    size_t msg_wlen = BM_RX_FRAME_LEN_ALIGNED_WLEN( len );

    // Allocate
    struct bm_rx_frame* frame = bm_rx_frame_alloc( pbuf, msg_wlen );

    // Create descriptor
    struct bm_rx_frame_desc desc = {
        .valid = 0,
        .busy = 0,
        .type = BM_RX_FRAME_TYPE_DEFAULT,
        .phy_idx = phy_idx,
        .data_len = len,
        .reserved = 0
    };

    // Finalize and commit
    int ret = bm_rx_frame_finalize( pbuf, frame, desc, data );

    return ret;
}

static inline uint32_t bm_rx_frame_get_total_wlen(const struct bm_rx_frame_desc desc)
{
	return BM_RX_FRAME_LEN_ALIGNED_WLEN(desc.data_len);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_BRISTLEMOUTH_BM_RX_FRAME_H_ */