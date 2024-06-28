/*******************************************************************************
 * File Name: app_bt_periodic_adv.c
 *
 * Description: This file consists of the bt periodic advertise sync function definitions
 *
 *******************************************************************************
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 *******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
/* FreeRTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "timers.h"

#include "stdio.h"
#include "app_bt_periodic_sync.h"
#include "cy_syslib.h"
#include "wiced_bt_trace.h"
#include "app_bt_utils.h"

/******************************************************************************
 *                                DEFINES
 ******************************************************************************/
#define HCI_BLE_EVENT_MASK_DEF_NEW "\x00\x00\x00\x00\x7F\xff\xff\xff"
#define PERIODIC_SYNC_START_TIME_VALUE      2000

/******************************************************************************
 *                                GLOBAL VARIABLES
 ******************************************************************************/
static TimerHandle_t periodic_sync_start_timer = NULL;
wiced_bt_device_address_t peer_bda = {0x20, 0x82, 0x9B, 0x01, 0x22, 0x33};

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
/**
 * Function Name:
 * periodic_sync_scan_result_cback
 *
 * Function Description:
 * @brief  This function is called when an ADV is received. This will typically
 * only sync with periodic adv
 *
 * @param[in]       p_scan_result  scan result
 * @param[in]       p_adv_data     Extra adv data
 *
 * @return void
 */
static void periodic_sync_scan_result_cback (wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    if(p_scan_result != NULL)
    {
        printf("[%s]  is_externed: %d, rssi:%i\r\n", __FUNCTION__, p_scan_result->is_extended, p_scan_result->rssi);
    }
}

/**
 * Function Name:
 * periodic_sync_ext_adv_callback
 *
 * Function Description:
 * @brief  Process Extended ADV events
 *
 * @param[in]       event      The event code
 * @param[in]       p_data     Event-specific supporting data
 *
 * @return void
 */
static void periodic_sync_ext_adv_callback (wiced_bt_ble_adv_ext_event_t event, wiced_bt_ble_adv_ext_event_data_t *p_data)
{
    uint8_t data_len = 0;
    printf("[%s]  event: %s\r\n", __FUNCTION__, app_bt_util_get_ext_adv_event_name(event));

    switch (event)
    {
        case WICED_BT_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT:
            printf("[%s] PERIODIC_ADV_SYNC_ESTABLISHED_EVENT\r\n", __FUNCTION__);
            wiced_bt_ble_scan (BTM_BLE_SCAN_TYPE_NONE, WICED_FALSE, NULL);
            break;

        case WICED_BT_BLE_PERIODIC_ADV_REPORT_EVENT:
            data_len = p_data->periodic_adv_report.data_len;
            if(data_len > 0)
            {
                printf("[%s] PERIODIC_ADV_REPORT_EVENT, len:%d, rssi:%i, content:%d...\r\n", __FUNCTION__, data_len,
                            p_data->periodic_adv_report.adv_rssi, p_data->periodic_adv_report.adv_data[0]);
            }
            else
            {
                printf("[%s] PERIODIC_ADV_REPORT_EVENT, len:%d, rssi:%i\r\n", __FUNCTION__, data_len,
                            p_data->periodic_adv_report.adv_rssi);
            }
            break;

        case WICED_BT_BLE_PERIODIC_ADV_SYNC_LOST_EVENT:
            printf("[%s] PERIODIC_ADV_SYNC_LOST_EVENT, re-sync again\r\n", __FUNCTION__);
            wiced_bt_ble_create_sync_to_periodic_adv (WICED_BT_BLE_SYNC_TO_PERIODIC_ADV_LIST,
                                                    APP_BT_PERIODIC_ADV_SET_ID, BLE_ADDR_PUBLIC, peer_bda, 0, APP_BT_PERIODIC_ADV_SYNC_TIMEOUT, 0);
            wiced_bt_ble_scan (BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_FALSE, periodic_sync_scan_result_cback);
            break;

        case WICED_BT_BLE_PAWR_SYNC_ESTABLISHED_EVENT:
            printf("[%s] PAWR_SYNC_ESTABLISHED_EVENT\r\n", __FUNCTION__);
            wiced_bt_ble_scan (BTM_BLE_SCAN_TYPE_NONE, WICED_FALSE, NULL);
            break;

        case WICED_BT_BLE_PAWR_IND_REPORT_EVENT:
            data_len = p_data->pawr_ind_report.data_length;
            if(data_len > 0)
            {
                printf("[%s] PAWR_IND_REPORT_EVENT, len:%d, rssi:%i, content:%d...\r\n", __FUNCTION__, p_data->pawr_ind_report.data_length,
                                p_data->pawr_ind_report.rssi, p_data->periodic_adv_report.adv_data[0]);
            }
            else
            {
                printf("[%s] PAWR_IND_REPORT_EVENT, len:%d, rssi:%i\r\n", __FUNCTION__, p_data->pawr_ind_report.data_length,
                                p_data->pawr_ind_report.rssi);
            }
            break;

        default:
            break;
    }
}

/**
 * Function Name:
 * app_bt_periodic_sync_start_cb
 *
 * Function Description:
 * @brief Callback to set parameters for the periodic synchronizer
 *
 * @param void
 *
 * @return void
 */
void app_bt_periodic_sync_start_cb(TimerHandle_t cb_params)
{
    wiced_bt_dev_status_t           status;
    wiced_bt_ble_ext_scan_config_t  scan_cfg;
    uint8_t periodic_adv_list = 0;
    uint8_t filter_list_size = 0;

    //WAR for btss may post PAWR event even though it didnt enable PAWR related at all
    extern unsigned int btsnd_hcic_ble_set_evt_mask (uint8_t* event_mask);
    btsnd_hcic_ble_set_evt_mask((uint8_t*)HCI_BLE_EVENT_MASK_DEF_NEW);

    wiced_bt_ble_observe (WICED_FALSE, 0, NULL);

    // Register for extended ADV events
    wiced_bt_ble_register_adv_ext_cback (periodic_sync_ext_adv_callback);

    wiced_bt_ble_update_scanner_filter_list(1, peer_bda, BLE_ADDR_PUBLIC);

    scan_cfg.scanning_phys = WICED_BT_BLE_EXT_ADV_PHY_1M;
    scan_cfg.duration      = 0;
    scan_cfg.period        = 0;
    // set extended scan parameters
    status = wiced_bt_ble_cache_ext_scan_config(&scan_cfg);
    printf("[%s] wiced_bt_ble_cache_ext_scan_config status: %d\r\n", __FUNCTION__, status);

    filter_list_size = wiced_bt_ble_get_filter_accept_list_size();
    printf("[%s] wiced_bt_ble_get_filter_accept_list_size list size: %d\r\n", __FUNCTION__, filter_list_size);

    periodic_adv_list = wiced_bt_ble_read_periodic_adv_list_size();
    printf("[%s] wiced_bt_ble_read_periodic_adv_list_size list size: %d\r\n", __FUNCTION__, periodic_adv_list);

    status = wiced_bt_ble_add_device_to_periodic_adv_list(BLE_ADDR_PUBLIC, peer_bda, APP_BT_PERIODIC_ADV_SET_ID);
    printf("[%s] wiced_bt_ble_add_device_to_periodic_adv_list status: %d\r\n", __FUNCTION__, status);

    status = wiced_bt_ble_create_sync_to_periodic_adv (WICED_BT_BLE_SYNC_TO_PERIODIC_ADV_LIST,
                                                        APP_BT_PERIODIC_ADV_SET_ID, BLE_ADDR_PUBLIC, peer_bda, 0, APP_BT_PERIODIC_ADV_SYNC_TIMEOUT, 0);

    printf("[%s] wiced_bt_ble_create_sync_to_periodic_adv BDA: %02x-%02x-%02x-%02x-%02x-%02x Result: %d\r\n", 
            __FUNCTION__, peer_bda[0], peer_bda[1], peer_bda[2], peer_bda[3], peer_bda[4], peer_bda[5], status);

    wiced_bt_ble_update_scanner_filter_policy(BTM_BLE_SCAN_POLICY_FILTER_ADV_RSP);
    // set extended scan enable
    status = wiced_bt_ble_scan (BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_FALSE, periodic_sync_scan_result_cback);

    printf("[%s] wiced_bt_ble_scan status: %d\r\n", __FUNCTION__, status);
}

/**
 * Function Name:
 * app_bt_periodic_sync_init
 *
 * Function Description:
 * @brief Initialize ble periodic sync, fire a 2s timer to start the periodic sync
 *
 * @param void
 *
 * @return void
 */
void app_bt_periodic_sync_init(void)
{
    periodic_sync_start_timer = xTimerCreate("Simulate Sensor Timer",
                                PERIODIC_SYNC_START_TIME_VALUE,
                                pdFALSE,
                                NULL ,
                                app_bt_periodic_sync_start_cb);

    if (periodic_sync_start_timer != NULL && pdPASS != xTimerStart(periodic_sync_start_timer, 10u))
    {
        printf("Failed to start periodic_sync_start_timer!\r\n");
        CY_ASSERT(0);
    }
}

/* [] END OF FILE */
