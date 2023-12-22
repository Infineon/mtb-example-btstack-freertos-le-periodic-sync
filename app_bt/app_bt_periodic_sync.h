/*******************************************************************************
 * File Name: app_bt_periodic_adv.h
 *
 * Description: This file consists of the bt periodic advertise function declarations
 *
 ********************************************************************************
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 *******************************************************************************/

#ifndef __APP_BT_PERIODIC_ADV_H__
#define __APP_BT_PERIODIC_ADV_H__

/*******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
#include "wiced_bt_ble.h"

/*******************************************************************************
 *                                Constants
 ******************************************************************************/
#define APP_BT_PERIODIC_ADV_HANDLE              (0x01)
#define APP_BT_PERIODIC_ADV_SET_ID              (0x01)
#define APP_BT_PERIODIC_ADV_SYNC_TIMEOUT        (500)
#define APP_BT_PERIODIC_ADV_INT_MIN             (0x00F0)
#define APP_BT_PERIODIC_ADV_INT_MAX             (0x0190)

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
void app_bt_periodic_sync_init(void);

#endif      /* __APP_BT_PERIODIC_ADV_H__ */

/* [] END OF FILE */
