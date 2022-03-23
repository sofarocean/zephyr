#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_UART_PIPE_H_
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_UART_PIPE_H_

#include <net/ieee802154_radio.h>

#define UART_PIPE_RADIO_15_4_FRAME_TYPE		0xF0

typedef enum upipe_802154_rx_error_t
{
	UPIPE_802154_RX_ERROR_INVALID_FRAME,
	UPIPE_802154_RX_ERROR_DELAYED_TIMEOUT,
	UPIPE_802154_RX_ERROR_INVALID_FCS,
	UPIPE_802154_RX_ERROR_INVALID_DEST_ADDR,
	UPIPE_802154_RX_ERROR_CATCHALL
} upipe_802154_rx_error_t;

struct upipe_context {
	struct net_if *iface;
	uint8_t mac_addr[8];
	bool stopped;
	/** RX specific attributes */
	uint8_t uart_pipe_buf[1];
	bool rx;
	uint8_t rx_len;
	uint8_t rx_off;
	uint8_t rx_buf[127];
	ieee802154_event_cb_t event_handler;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_UART_PIPE_H_ */