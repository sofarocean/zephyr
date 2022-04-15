#ifndef ZEPHYR_DRIVERS_IEEE802154_BM_SERIAL_H_
#define ZEPHYR_DRIVERS_IEEE802154_BM_SERIAL_H_

#include <net/ieee802154_radio.h>

typedef enum ieee802154_bm_serial_rx_error_t
{
	IEEE802154_BM_SERIAL_RX_ERROR_INVALID_FRAME,
	IEEE802154_BM_SERIAL_RX_ERROR_DELAYED_TIMEOUT,
	IEEE802154_BM_SERIAL_RX_ERROR_INVALID_FCS,
	IEEE802154_BM_SERIAL_RX_ERROR_INVALID_DEST_ADDR,
	IEEE802154_BM_SERIAL_RX_ERROR_CATCHALL
} ieee802154_bm_serial_rx_error_t;

typedef struct ieee802154_bm_serial_context_t
{
	struct net_if *iface;
	uint8_t mac_addr[8];
	bool stopped;
	ieee802154_event_cb_t event_handler;
} ieee802154_bm_serial_context_t;

#endif /* ZEPHYR_DRIVERS_IEEE802154_BM_SERIAL_H_ */