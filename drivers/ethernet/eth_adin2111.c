#define DT_DRV_COMPAT analog_adin2111

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_adin2111, CONFIG_ETHERNET_LOG_LEVEL);

#include "adin2111/adin2111.h"
#include "adin2111/bsp/adi_bsp.h"
#include "adin2111/adi_mac.h"
#include "eth.h"
#include "eth_adin2111_priv.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/random/rand32.h>

#include <ethernet/eth_stats.h>

#define QUEUE_NUM_ENTRIES (32)

static adin2111_DeviceStruct_t dev;
static adin2111_DeviceHandle_t hDevice = &dev;
static uint8_t dev_mem[ADIN2111_DEVICE_SIZE];

static adin2111_DriverConfig_t drvConfig = {
    .pDevMem = (void *)dev_mem,
    .devMemSize = sizeof(dev_mem),
    .fcsCheckEn = false,
};

typedef struct {
    adin2111_Port_e port;
    adi_eth_BufDesc_t *pBufDesc;
    bool sent;
} queue_entry_t;

typedef struct {
    uint32_t head;
    uint32_t tail;
    bool full;
    queue_entry_t entries[QUEUE_NUM_ENTRIES];
} queue_t;

DMA_BUFFER_ALIGN(static uint8_t txQueueBuf[QUEUE_NUM_ENTRIES][MAX_FRAME_BUF_SIZE], 4);
DMA_BUFFER_ALIGN(static uint8_t rxQueueBuf[QUEUE_NUM_ENTRIES][MAX_FRAME_BUF_SIZE], 4);
static adi_eth_BufDesc_t txBufDesc[QUEUE_NUM_ENTRIES];
static adi_eth_BufDesc_t rxBufDesc[QUEUE_NUM_ENTRIES];

queue_t txQueue;
queue_t rxQueue;

static void adin2111_main_queue_init(queue_t *pQueue, adi_eth_BufDesc_t bufDesc[QUEUE_NUM_ENTRIES],
                     uint8_t buf[QUEUE_NUM_ENTRIES][MAX_FRAME_BUF_SIZE]);
static uint32_t adin2111_main_queue_available(queue_t *pQueue);
static bool adin2111_main_queue_is_full(queue_t *pQueue);
static bool adin2111_main_queue_is_empty(queue_t *pQueue);
static queue_entry_t *adin2111_main_queue_tail(queue_t *pQueue);
static queue_entry_t *adin2111_main_queue_head(queue_t *pQueue);
static void adin2111_main_queue_add(queue_t *pQueue, adin2111_Port_e port,
                    adi_eth_BufDesc_t *pBufDesc, adi_eth_Callback_t cbFunc);
static void adin2111_main_queue_add_desc(queue_t *pQueue, adin2111_Port_e port,
                     adi_eth_BufDesc_t *pBufDesc);
static void adin2111_main_queue_remove(queue_t *pQueue);

static void adin2111_rx_cb(void *pCBParam, uint32_t Event, void *pArg);
static void adin2111_tx_cb(void *pCBParam, uint32_t Event, void *pArg);
static void adin2111_link_change_cb(void *pCBParam, uint32_t Event, void *pArg);

static void adin2111_service_thread(const struct device *dev);
static int adin2111_tx(const struct device *dev, struct net_pkt *pkt);
static void adin2111_iface_init(struct net_if *iface);
static enum ethernet_hw_caps adin2111_get_capabilities(const struct device *dev);
static int adin2111_hw_start(const struct device *dev);
static int adin2111_hw_stop(const struct device *dev);
static void adin2111_random_mac(uint8_t *mac_addr);
static int adin2111_init(const struct device *dev);

K_SEM_DEFINE(tx_rdy, 0, 1);
K_SEM_DEFINE(rx_rdy, 0, 1);

struct k_poll_event adin_eth_events[2] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &tx_rdy, 0),

    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &rx_rdy, 0),
};

static const struct gpio_dt_spec user0 = GPIO_DT_SPEC_GET(DT_ALIAS(user0), gpios);
static const struct gpio_dt_spec user1 = GPIO_DT_SPEC_GET(DT_ALIAS(user1), gpios);
static const struct gpio_dt_spec user4 = GPIO_DT_SPEC_GET(DT_ALIAS(user4), gpios);

/* ====================================================================================================
    Main Queue Functions borrowed from ADIN2111 Example
   ====================================================================================================
 */

static void adin2111_main_queue_init(queue_t *pQueue, adi_eth_BufDesc_t bufDesc[QUEUE_NUM_ENTRIES],
                     uint8_t buf[QUEUE_NUM_ENTRIES][MAX_FRAME_BUF_SIZE])
{
    pQueue->head = 0;
    pQueue->tail = 0;
    pQueue->full = false;
    for (uint32_t i = 0; i < QUEUE_NUM_ENTRIES; i++) {
        pQueue->entries[i].pBufDesc = &bufDesc[i];
        pQueue->entries[i].pBufDesc->pBuf = &buf[i][0];
        pQueue->entries[i].pBufDesc->bufSize = MAX_FRAME_BUF_SIZE;
        pQueue->entries[i].pBufDesc->egressCapt = ADI_MAC_EGRESS_CAPTURE_NONE;
    }
}

static uint32_t adin2111_main_queue_available(queue_t *pQueue)
{
    if (pQueue->full) {
        return 0;
    }

    uint32_t n = (pQueue->head + QUEUE_NUM_ENTRIES - pQueue->tail) % QUEUE_NUM_ENTRIES;
    n = QUEUE_NUM_ENTRIES - n;

    return n;
}

static bool adin2111_main_queue_is_full(queue_t *pQueue)
{
    return pQueue->full;
}

static bool adin2111_main_queue_is_empty(queue_t *pQueue)
{
    bool isEmpty = !pQueue->full && (pQueue->head == pQueue->tail);

    return isEmpty;
}

static queue_entry_t *adin2111_main_queue_tail(queue_t *pQueue)
{
    queue_entry_t *p = NULL;

    if (!adin2111_main_queue_is_empty(pQueue)) {
        p = &pQueue->entries[pQueue->tail];
    }

    return p;
}

static queue_entry_t *adin2111_main_queue_head(queue_t *pQueue)
{
    queue_entry_t *p = NULL;

    if (!adin2111_main_queue_is_full(pQueue)) {
        p = &pQueue->entries[pQueue->head];
    }

    return p;
}

static void adin2111_main_queue_add(queue_t *pQueue, adin2111_Port_e port,
                    adi_eth_BufDesc_t *pBufDesc, adi_eth_Callback_t cbFunc)
{
    queue_entry_t *pEntry;

    pEntry = adin2111_main_queue_head(pQueue);

    if (adin2111_main_queue_available(pQueue) == 1) {
        pQueue->full = true;
    }

    pEntry->port = port;
    pEntry->pBufDesc->bufSize = pBufDesc->bufSize;
    pEntry->pBufDesc->cbFunc = cbFunc;
    pEntry->pBufDesc->trxSize = pBufDesc->trxSize;
    memcpy(pEntry->pBufDesc->pBuf, pBufDesc->pBuf, pBufDesc->trxSize);

    pEntry->sent = false;
    pQueue->head = (pQueue->head + 1) % QUEUE_NUM_ENTRIES;
}

static void adin2111_main_queue_remove(queue_t *pQueue)
{
    pQueue->full = false;

    pQueue->tail = (pQueue->tail + 1) % QUEUE_NUM_ENTRIES;
}

/* ====================================================================================================
    Relevant Callbacks
   ====================================================================================================
 */

static void adin2111_rx_cb(void *pCBParam, uint32_t Event, void *pArg)
{
    adi_eth_BufDesc_t *pBufDesc;

    gpio_pin_configure_dt(&user0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&user0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&user0, GPIO_OUTPUT_ACTIVE);

    pBufDesc = (adi_eth_BufDesc_t *)pArg;
    LOG_DBG("Received Frame on Port: %d!", (adin2111_Port_e)pBufDesc->port);
    k_sem_give(adin_eth_events[1].sem);
}

static void adin2111_tx_cb(void *pCBParam, uint32_t Event, void *pArg)
{
    adi_eth_BufDesc_t *pBufDesc;

    gpio_pin_configure_dt(&user1, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&user1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&user1, GPIO_OUTPUT_ACTIVE);

    pBufDesc = (adi_eth_BufDesc_t *)pArg;
    // LOG_DBG("Sent Frame on Port: %d!", (adin2111_Port_e)pBufDesc->port);
}

static void adin2111_link_change_cb(void *pCBParam, uint32_t Event, void *pArg)
{
    adi_mac_StatusRegisters_t statusRegisters = *(adi_mac_StatusRegisters_t *)pArg;

    /* The port where the link status changed happened can be determined by */
    /* examining the values of the masked status registers. Then the link   */
    /* status can be read from the actual PHY registers. */

    LOG_INF("Link Change Callback entered");

    (void)statusRegisters;
}

/* ====================================================================================================
    Zephyr Integration functions
   ====================================================================================================
 */

static void adin2111_service_thread(const struct device *dev)
{
    adi_eth_Result_e result;
    queue_entry_t *pEntry;
    struct net_buf *pkt_buf = NULL;
    struct net_pkt *pkt;
    uint16_t read_len;
    uint16_t reader;
    struct adin2111_runtime *ctx = dev->data;
    const struct adin2111_config *config = dev->config;
    int rc;

    k_thread_custom_data_set((void *) &user4);

    while (1) {
        rc = k_poll(adin_eth_events, 2, K_USEC(10));

        /* Check if timed out */
        if (rc != 0) {
            continue;
        }

        if (adin_eth_events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
            k_sem_take(adin_eth_events[0].sem, K_NO_WAIT);
            if (!adin2111_main_queue_is_empty(&txQueue)) {
                pEntry = adin2111_main_queue_tail(&txQueue);
                if ((pEntry != NULL) && (!pEntry->sent)) {
                    pEntry->sent = true;
                    result = adin2111_SubmitTxBuffer(hDevice, (adin2111_TxPort_e)pEntry->port, pEntry->pBufDesc);
                    //result = adin2111_SubmitTxBuffer(hDevice, (adin2111_TxPort_e)pEntry->port, pEntry->pBufDesc);
                    //result = adin2111_SubmitTxBuffer(hDevice, (adin2111_TxPort_e)pEntry->port, pEntry->pBufDesc);
                    //result = adin2111_SubmitTxBuffer(hDevice, (adin2111_TxPort_e)pEntry->port, pEntry->pBufDesc);
                    if (result == ADI_ETH_SUCCESS) {
                        LOG_DBG("Submitted TX Buf to ADIN2111");
                        adin2111_main_queue_remove(&txQueue);
                    }
                }
            }
            adin2111_main_queue_remove(&txQueue);
        }

        adin_eth_events[0].signal->signaled = 0;
        adin_eth_events[0].state = K_POLL_STATE_NOT_READY;

        if (adin_eth_events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
            k_sem_take(adin_eth_events[1].sem, K_NO_WAIT);
            if (!adin2111_main_queue_is_empty(&rxQueue)) {
                pEntry = adin2111_main_queue_tail(&rxQueue);

                // if( !pEntry ) {
                // 	LOG_ERR( "Shouldn't happen: Null adin queue entry" );
                // 	goto out;
                // }

                // pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, pEntry->pBufDesc->trxSize, AF_UNSPEC, 0, K_MSEC(config->timeout));
                // if (!pkt) {
                // 	LOG_ERR("Failed to obtain RX packet buffer");
                // 	goto out;
                // }

                // if (net_pkt_write(pkt, pEntry->pBufDesc->pBuf, pEntry->pBufDesc->trxSize )) {
                // 	LOG_ERR("Unable to copy frame into pkt");
                // 	net_pkt_unref(pkt);
                // 	goto out;
                // }

                // int res = net_recv_data(ctx->iface, pkt);
                // if (res < 0) {
                // 	LOG_ERR("Error receiving data: %d", res );
                // 	net_pkt_unref(pkt);
                // 	goto out;
                // }

out:
                /* Put the buffer back into queue and re-submit to the ADIN2111 driver */
                adin2111_main_queue_remove(&rxQueue);
                adin2111_main_queue_add(&rxQueue, ADIN2111_PORT_1, pEntry->pBufDesc, adin2111_rx_cb);
                result = adin2111_SubmitRxBuffer(hDevice, pEntry->pBufDesc);
                if (result == ADI_ETH_SUCCESS) {
                    LOG_DBG("Submitted RX Buf to ADIN2111");
                }
            }
        }

        adin_eth_events[1].signal->signaled = 0;
        adin_eth_events[1].state = K_POLL_STATE_NOT_READY;
    }
}

static int adin2111_tx(const struct device *dev, struct net_pkt *pkt)
{
    int result = 0;
    queue_entry_t *pEntry;

    if (!adin2111_main_queue_is_full(&txQueue)) {
        pEntry = adin2111_main_queue_head(&txQueue);
        pEntry->pBufDesc->trxSize = net_pkt_get_len(pkt);

        if (net_pkt_read(pkt, pEntry->pBufDesc->pBuf, pEntry->pBufDesc->trxSize)) {
            LOG_ERR("Failed to read net pkt");
            return -EIO;
        } else {
            LOG_DBG("Submitting packet to ADIN2111 for TX");
        }

        pEntry->pBufDesc->cbFunc = adin2111_tx_cb;

        /* TODO: Send out of PORT1 for now */
        adin2111_main_queue_add(&txQueue, ADIN2111_PORT_1, pEntry->pBufDesc, adin2111_tx_cb);
        k_sem_give(adin_eth_events[0].sem);
    } else {
        LOG_ERR("ADIN2111 Driver Tx Queue is Full");
    }
    return result;
}

static void adin2111_iface_init(struct net_if *iface)
{
    const struct device *dev = net_if_get_device(iface);
    struct adin2111_runtime *ctx = dev->data;

    net_if_set_link_addr(iface, ctx->mac_addr, sizeof(ctx->mac_addr), NET_LINK_ETHERNET);

    if (!ctx->iface) {
        ctx->iface = iface;
    }

    ethernet_init(iface);
}

static enum ethernet_hw_caps adin2111_get_capabilities(const struct device *dev)
{
    ARG_UNUSED(dev);
    return ETHERNET_LINK_10BASE_T;
}

static int adin2111_hw_start(const struct device *dev)
{
    return adin2111_Enable(hDevice);
}

static int adin2111_hw_stop(const struct device *dev)
{
    return adin2111_Disable(hDevice);
}

static struct ethernet_api adin2111_api_funcs = {
    .iface_api.init = adin2111_iface_init,
    .get_capabilities = adin2111_get_capabilities,
    // .set_config = adin2111_set_config,
    .start = adin2111_hw_start,
    .stop = adin2111_hw_stop,
    .send = adin2111_tx,
};

static void adin2111_random_mac(uint8_t *mac_addr)
{
    uint32_t entropy = sys_rand32_get();

    mac_addr[0] = ADIN_MAC_ADDR_0;
    mac_addr[1] = ADIN_MAC_ADDR_1;
    mac_addr[2] = ADIN_MAC_ADDR_2;
    mac_addr[3] = (entropy >> 16) & 0xff;
    mac_addr[4] = (entropy >> 8) & 0xff;
    mac_addr[5] = (entropy >> 0) & 0xff;

    LOG_INF("Generated MAC Address: %x:%x:%x:%x:%x:%x", mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5]);
}

static int adin2111_init(const struct device *dev) {
    adi_eth_Result_e result;
    adi_mac_AddressRule_t addrRule;
    const struct adin2111_config *config = dev->config;
    struct adin2111_runtime *ctx = dev->data;

    LOG_INF("Initializing ADIN2111");

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

    /* Initialize BSP to kickoff thread to service GPIO and SPI DMA interrupts */
    if (BSP_Init()) {
        LOG_ERR("BSP Init failed");
        while (1);
    }

    /* ADIN2111 Init process */
    result = adin2111_Init(hDevice, &drvConfig);
    if (result != ADI_ETH_SUCCESS) {
        LOG_ERR("ADIN2111 failed initialization");
    }

    /* generate random MAC addresses for each port/phy */
    addrRule.VALUE16 = 0x0000;

    addrRule.APPLY2PORT1 = 1;
    addrRule.APPLY2PORT2 = 1;
    addrRule.TO_HOST = 1;

    adin2111_random_mac(ctx->mac_addr);
    result = adin2111_AddAddressFilter(hDevice, ctx->mac_addr, NULL, addrRule);
    if (result != ADI_ETH_SUCCESS) {
        LOG_ERR("ADIN2111 failed to add address filter");
    }

    /* Register Callback for link change */
    result = adin2111_RegisterCallback(hDevice, adin2111_link_change_cb,
                       ADI_MAC_EVT_LINK_CHANGE);
    if (result != ADI_ETH_SUCCESS) {
        LOG_ERR("ADIN2111 failed to register Link Change callback");
    }

    /* Prepare Rx buffers */
    adin2111_main_queue_init(&txQueue, txBufDesc, txQueueBuf);
    adin2111_main_queue_init(&rxQueue, rxBufDesc, rxQueueBuf);
    for (uint32_t i = 0; i < QUEUE_NUM_ENTRIES; i++) {
        rxQueue.entries[i].pBufDesc->cbFunc = adin2111_rx_cb;

        /* add to queue (port set to 1 for now) */
        adin2111_main_queue_add(&rxQueue, ADIN2111_PORT_1, rxQueue.entries[i].pBufDesc, adin2111_rx_cb);

        /* Submit the RX buffer ahead of time */
        adin2111_SubmitRxBuffer(hDevice, rxQueue.entries[i].pBufDesc);
    }

    /* Confirm device configuration */
    result = adin2111_SyncConfig(hDevice);
    if (result != ADI_ETH_SUCCESS) {
        LOG_ERR("ADIN2111 failed Sync Config");

    }

    k_thread_create(&ctx->thread, ctx->thread_stack,
            CONFIG_ETH_ADIN2111_SERVICE_THREAD_STACK_SIZE,
            (k_thread_entry_t)adin2111_service_thread, (void *)dev, NULL, NULL,
            K_PRIO_COOP(CONFIG_ETH_ADIN2111_SERVICE_THREAD_PRIO), 0, K_NO_WAIT);
    k_thread_name_set(&ctx->thread, "adin2111_service");

    return 0;
}

static struct adin2111_runtime adin2111_0_runtime = {
    .generate_mac = adin2111_random_mac,
    .tx_sem = Z_SEM_INITIALIZER(adin2111_0_runtime.tx_sem, 1, UINT_MAX),
    .int_sem = Z_SEM_INITIALIZER(adin2111_0_runtime.int_sem, 0, UINT_MAX),
};

static const struct adin2111_config adin2111_0_config = {
    .spi = SPI_DT_SPEC_GET(DT_NODELABEL(spi_adin2111), SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),
    .interrupt = GPIO_DT_SPEC_INST_GET(0, int_gpios),
    .reset = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios, {0}),
    .timeout = CONFIG_ETH_ADIN2111_TIMEOUT,
};

ETH_NET_DEVICE_DT_INST_DEFINE(0, adin2111_init, NULL, &adin2111_0_runtime, &adin2111_0_config,
                  CONFIG_ETH_INIT_PRIORITY, &adin2111_api_funcs, NET_ETH_MTU);