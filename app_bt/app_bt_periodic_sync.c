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
#define MAX_EXT_SCAN_LEN               (300U)

/******************************************************************************
 *                                GLOBAL VARIABLES
 ******************************************************************************/
static TimerHandle_t periodic_sync_start_timer = NULL;
wiced_bt_device_address_t peer_bda = {0x20, 0x82, 0x9B, 0x01, 0x22, 0x33};

wiced_ble_ext_scan_params_t scan_params =
{
    .own_addr_type = BLE_ADDR_PUBLIC,
    .scanning_phys = WICED_BLE_EXT_ADV_PHY_1M_BIT,
    .scan_filter_policy = WICED_BLE_EXT_SCAN_BASIC_UNFILTERED_SP,
    .sp_1m.scan_type = BTM_BLE_SCAN_MODE_PASSIVE,
    .sp_1m.scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,
    .sp_1m.scan_window = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW
};

wiced_ble_ext_scan_enable_params_t scan_enable =
{
    .scan_duration =0,
    .scan_period = 0,
};

wiced_ble_padv_create_sync_params_t sync_par =
{
    .options = WICED_BLE_PADV_CREATE_SYNC_OPTION_IGNORE_PA_LIST,
    .adv_sid = APP_BT_PERIODIC_ADV_SET_ID,
    .adv_addr_type = BLE_ADDR_PUBLIC,
    .adv_addr = {0x20, 0x82, 0x9B, 0x01, 0x22, 0x33},
    .skip= 0,
    .sync_timeout = APP_BT_PERIODIC_ADV_SYNC_TIMEOUT,
    .sync_cte_type = 0,
};

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

static void ext_scan_result_cback(wiced_ble_ext_scan_results_t *p_scan_result,
        uint16_t adv_data_len,
        uint8_t *p_adv_data)
{
    if (p_scan_result != NULL)
    {
        printf("[%s]  rssi:%i\n", __FUNCTION__, p_scan_result->rssi);
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
static void periodic_sync_ext_adv_callback (wiced_ble_ext_adv_event_t event, wiced_ble_ext_adv_event_data_t *p_data)
{
    uint8_t data_len = 0;
    printf("[%s]  event: %s\r\n", __FUNCTION__, app_bt_util_get_ext_adv_event_name(event));

    switch (event)
    {
        case WICED_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT:
            printf("[%s] PERIODIC_ADV_SYNC_ESTABLISHED_EVENT\r\n", __FUNCTION__);
            wiced_ble_ext_scan_enable(0,&scan_enable);
            break;

        case WICED_BLE_PERIODIC_ADV_REPORT_EVENT:
            data_len = p_data->periodic_adv_report.data_length;
            if(data_len > 0)
            {
                printf("[%s] PERIODIC_ADV_REPORT_EVENT, len:%d, rssi:%i,content:%d...\r\n", __FUNCTION__, data_len,
                            p_data->periodic_adv_report.rssi,p_data->
                            periodic_adv_report.p_data[0]);
            }
            else
            {
                printf("[%s] PERIODIC_ADV_REPORT_EVENT, len:%d, rssi:%i\r\n", __FUNCTION__, data_len,
                            p_data->periodic_adv_report.rssi);
            }
            break;

        case WICED_BLE_PERIODIC_ADV_SYNC_LOST_EVENT:
            printf("[%s] PERIODIC_ADV_SYNC_LOST_EVENT, re-sync again\r\n", __FUNCTION__);
            memcpy(sync_par.adv_addr, peer_bda, BD_ADDR_LEN);
            wiced_ble_padv_create_sync (&sync_par);
            wiced_ble_ext_scan_enable(1,&scan_enable);

            break;

        case WICED_BT_BLE_PAWR_SUBEVENT_DATA_REQ_EVENT:
            printf("[%s] PAWR_SYNC_ESTABLISHED_EVENT\n", __FUNCTION__);
            wiced_ble_ext_scan_enable(0,&scan_enable);
            break;

        case WICED_BT_BLE_PAWR_RSP_REPORT_EVENT:
            data_len = p_data->pawr_rsp_report.data_length;
            if(data_len > 0)
            {
                printf("[%s] PAWR_RSP_REPORT_EVENT, len:%d, rssi:%i, content:%d...\r\n", __FUNCTION__, p_data->pawr_rsp_report.data_length,
                                p_data->pawr_rsp_report.rssi, p_data->pawr_rsp_report.p_data[0]);
            }
            else
            {
                printf("[%s] PAWR_RSP_REPORT_EVENT, len:%d, rssi:%i\r\n", __FUNCTION__, p_data->pawr_rsp_report.data_length,
                                p_data->pawr_rsp_report.rssi);
            }
            break;

        default:
            break;
    }
}
void app_bt_start_periodic_sync(void)
{
    wiced_bt_dev_status_t status;
    memcpy(sync_par.adv_addr, peer_bda, BD_ADDR_LEN);
    status = wiced_ble_padv_add_device_to_list(BLE_ADDR_PUBLIC, peer_bda,
           APP_BT_PERIODIC_ADV_SET_ID);

    if (WICED_SUCCESS != status)
    {
        printf("Error adding device to Periodic Advertiser list\n");
        CY_ASSERT(0);
    }

    status = wiced_ble_padv_create_sync (&sync_par);
    if (WICED_SUCCESS == status)
    {
        printf("[%s] wiced_ble_padv_create_sync Result: %d BDA: \n",
            __FUNCTION__, status);
    }
    else
    {
        printf("[%s] wiced_ble_padv_create_sync Result: %d BDA: \n",
            __FUNCTION__, status);
        CY_ASSERT(0);
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
    wiced_bt_dev_status_t status;

    /* Set extended scan register callback */
    status = wiced_ble_ext_scan_register_cb(ext_scan_result_cback);
    if (WICED_SUCCESS != status)
    {
        printf("Error extended scan register callback\n");
        CY_ASSERT(0);
    }

    /* Register for extended ADV events */
    wiced_ble_ext_adv_register_cback(periodic_sync_ext_adv_callback);

    /* set extended scan parameters */
    status = wiced_ble_ext_scan_set_params(&scan_params);
    if (WICED_SUCCESS != status)
    {
        printf("Error setting extended scan parameters\n");
        CY_ASSERT(0);
    }

    /* Disable extended scan */
    status = wiced_ble_ext_scan_enable(0,&scan_enable);
    if (WICED_SUCCESS != status)
    {
        printf("Error disabling extended scan\n");
        CY_ASSERT(0);
    }

    /* Start periodic sync */
    app_bt_start_periodic_sync();

    app_bt_util_print_bd_address(peer_bda);

    /* set extended scan enable */
    status = wiced_ble_ext_scan_enable(1,&scan_enable);
    if (WICED_SUCCESS == status)
    {
        printf("[%s] wiced_ble_ext_scan_enable status:"
            "%d\n", __FUNCTION__, status);
    }
    else
    {
        printf("[%s] wiced_ble_ext_scan_enable status:"
            "%d\n", __FUNCTION__, status);
        CY_ASSERT(0);
    }
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
