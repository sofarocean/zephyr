/* main.c - OpenThread NCP */

/*
 * Copyright (c) 2020 Tridonic GmbH & Co KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
#include <net/openthread.h>
#include <openthread/dataset.h>

LOG_MODULE_REGISTER(ot_br, LOG_LEVEL_DBG);

#define APP_BANNER "***** OpenThread NCP on Zephyr %s *****"

void main(void)
{
    struct openthread_context* ot_ctx = openthread_get_default_context();
    otInstance* ot_instance = ot_ctx->instance;

    LOG_INF( "Starting OT Manually with params:" );
    LOG_INF( "Network Name: %s", log_strdup(CONFIG_OPENTHREAD_NETWORK_NAME) );
    LOG_INF( "Channel: %d", CONFIG_OPENTHREAD_CHANNEL );
    LOG_INF( "PANID: %d", CONFIG_OPENTHREAD_PANID );
    LOG_INF( "XPANID: %s", log_strdup(CONFIG_OPENTHREAD_XPANID) );
    LOG_INF( "Network Key: %s", log_strdup(CONFIG_OPENTHREAD_NETWORKKEY) );

    // Set up new operational dataset with configured params
    otOperationalDataset op_dataset;
    otDatasetCreateNewNetwork(ot_instance, &op_dataset);
    op_dataset.mChannel = CONFIG_OPENTHREAD_CHANNEL;
    op_dataset.mPanId = CONFIG_OPENTHREAD_PANID;
    otNetworkNameFromString(&op_dataset.mNetworkName, CONFIG_OPENTHREAD_NETWORK_NAME);
    net_bytes_from_str(op_dataset.mExtendedPanId.m8, 8, (char *)CONFIG_OPENTHREAD_XPANID);
    net_bytes_from_str(op_dataset.mNetworkKey.m8, OT_NETWORK_KEY_SIZE,(char *)CONFIG_OPENTHREAD_NETWORKKEY);

    op_dataset.mMeshLocalPrefix.m8[0] = 0xfd;
    memcpy(&op_dataset.mMeshLocalPrefix.m8[1], op_dataset.mExtendedPanId.m8, 5);
    op_dataset.mMeshLocalPrefix.m8[6] = 0x00;
    op_dataset.mMeshLocalPrefix.m8[7] = 0x00;

    // Commit dataset as active
    otDatasetSetActive(ot_instance, &op_dataset);

    // Start OT
    openthread_start(ot_ctx);
    LOG_INF( "Started OT" );
}
