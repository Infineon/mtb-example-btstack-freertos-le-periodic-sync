/*******************************************************************************
 * File Name: app_bt_utils.c
 *
 * Description: This file consists of the utility function definitions that will
 *              help debugging and developing the applications easier with much
 *              more meaningful information.
 *
 *******************************************************************************
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 *******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 ******************************************************************************/
#include "app_bt_utils.h"
#include "wiced_bt_dev.h"
#include "cybt_platform_trace.h"
#include "cycfg_gap.h"
#include "cy_efuse.h"

/******************************************************************************
 *                                DEFINES
 ******************************************************************************/

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
/**
 * Function Name:
 * app_bt_util_get_btm_event_name
 *
 * Function Description:
 * @brief  The function converts the wiced_bt_management_evt_t enum value to its
 *         corresponding string literal. This will help the programmer to debug
 *         easily with log traces without navigating through the source code.
 *
 * @param event: Bluetooth management event type
 *
 * @return char*: String corresponding to the wiced_bt_management_evt_t event
 */
const char* app_bt_util_get_btm_event_name(wiced_bt_management_evt_t event)
{
    switch ((int)event)
    {
        CASE_RETURN_STR(BTM_ENABLED_EVT)
        CASE_RETURN_STR(BTM_DISABLED_EVT)
        CASE_RETURN_STR(BTM_POWER_MANAGEMENT_STATUS_EVT)
        CASE_RETURN_STR(BTM_PIN_REQUEST_EVT)
        CASE_RETURN_STR(BTM_USER_CONFIRMATION_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PASSKEY_NOTIFICATION_EVT)
        CASE_RETURN_STR(BTM_PASSKEY_REQUEST_EVT)
        CASE_RETURN_STR(BTM_KEYPRESS_NOTIFICATION_EVT)
        CASE_RETURN_STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT)
        CASE_RETURN_STR(BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PAIRING_COMPLETE_EVT)
        CASE_RETURN_STR(BTM_ENCRYPTION_STATUS_EVT)
        CASE_RETURN_STR(BTM_SECURITY_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SECURITY_FAILED_EVT)
        CASE_RETURN_STR(BTM_SECURITY_ABORTED_EVT)
        CASE_RETURN_STR(BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT)
        CASE_RETURN_STR(BTM_REMOTE_OOB_DATA_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT)
        CASE_RETURN_STR(BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT)
        CASE_RETURN_STR(BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT)
        CASE_RETURN_STR(BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT)
        CASE_RETURN_STR(BTM_BLE_SCAN_STATE_CHANGED_EVT)
        CASE_RETURN_STR(BTM_BLE_ADVERT_STATE_CHANGED_EVT)
        CASE_RETURN_STR(BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT)
        CASE_RETURN_STR(BTM_SCO_CONNECTED_EVT)
        CASE_RETURN_STR(BTM_SCO_DISCONNECTED_EVT)
        CASE_RETURN_STR(BTM_SCO_CONNECTION_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SCO_CONNECTION_CHANGE_EVT)
        CASE_RETURN_STR(BTM_BLE_CONNECTION_PARAM_UPDATE)
        CASE_RETURN_STR(BTM_BLE_DATA_LENGTH_UPDATE_EVENT)
#ifdef CYW20819A1
        CASE_RETURN_STR(BTM_BLE_PHY_UPDATE_EVT)
#endif
    }

    return "UNKNOWN_EVENT";
}

/**
 * Function Name:
 * app_bt_util_get_ext_adv_event_name
 *
 * Function Description:
 * @brief  The function converts the wiced_bt_ble_adv_ext_event_t enum value to its
 *         corresponding string literal. This will help the programmer to debug
 *         easily with log traces without navigating through the source code.
 *
 * @param event: extern adv event type
 *
 * @return char*: String corresponding to the wiced_bt_ble_adv_ext_event_t event
 */
const char* app_bt_util_get_ext_adv_event_name(wiced_ble_ext_adv_event_t event)
{
    switch ((int)event)
    {
        CASE_RETURN_STR(WICED_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT)
        CASE_RETURN_STR(WICED_BLE_PERIODIC_ADV_REPORT_EVENT)
        CASE_RETURN_STR(WICED_BLE_PERIODIC_ADV_SYNC_LOST_EVENT)
        CASE_RETURN_STR(WICED_BLE_PERIODIC_ADV_SYNC_TRANSFER_EVENT)

        CASE_RETURN_STR(WICED_BT_BLE_ADV_SET_TERMINATED_EVENT)
        CASE_RETURN_STR(WICED_BT_BLE_SCAN_REQUEST_RECEIVED_EVENT)
        CASE_RETURN_STR(WICED_BT_BLE_BIGINFO_ADV_REPORT_EVENT)
        CASE_RETURN_STR(WICED_BLE_EXT_COMMAND_CMPLT_EVENT)

        CASE_RETURN_STR(WICED_BT_BLE_PAWR_SUBEVENT_DATA_REQ_EVENT)
        CASE_RETURN_STR(WICED_BT_BLE_PAWR_RSP_REPORT_EVENT)
    }

    return "UNKNOWN_EVENT";
}

/**
 * Function Name:
 * app_bt_util_get_btm_advert_mode_name
 *
 * Function Description:
 * @brief  The function converts the wiced_bt_ble_advert_mode_t enum value to its
 *        corresponding string literal. This will help the programmer to debug
 *        easily with log traces without navigating through the source code.
 *
 * @param mode:   Bluetooth advertisement mode type
 *
 * @return char*: String corresponding to the wiced_bt_ble_advert_mode_t mode
 */
const char* app_bt_util_get_btm_advert_mode_name(wiced_bt_ble_advert_mode_t mode)
{

    switch ((int)mode)
    {

        CASE_RETURN_STR(BTM_BLE_ADVERT_OFF)
        CASE_RETURN_STR(BTM_BLE_ADVERT_DIRECTED_HIGH)
        CASE_RETURN_STR(BTM_BLE_ADVERT_DIRECTED_LOW)
        CASE_RETURN_STR(BTM_BLE_ADVERT_UNDIRECTED_HIGH)
        CASE_RETURN_STR(BTM_BLE_ADVERT_UNDIRECTED_LOW)
        CASE_RETURN_STR(BTM_BLE_ADVERT_NONCONN_HIGH)
        CASE_RETURN_STR(BTM_BLE_ADVERT_NONCONN_LOW)
        CASE_RETURN_STR(BTM_BLE_ADVERT_DISCOVERABLE_HIGH)
        CASE_RETURN_STR(BTM_BLE_ADVERT_DISCOVERABLE_LOW)

    }

    return "UNKNOWN_MODE";
}

/**
 * Function Name:
 * app_bt_util_get_gatt_disconn_reason_name
 *
 * Function Description:
 * @brief  The function converts the wiced_bt_gatt_disconn_reason_t enum value to
 *        its corresponding string literal. This will help the programmer to debug
 *        easily with log traces without navigating through the source code.
 *
 * @param reason:   GATT Disconnection reason
 *
 * @return char*: String corresponding to the wiced_bt_gatt_disconn_reason_t reason
 */
const char* app_bt_util_get_gatt_disconn_reason_name(wiced_bt_gatt_disconn_reason_t reason)
{

    switch ((int)reason)
    {

        CASE_RETURN_STR(GATT_CONN_UNKNOWN)
        CASE_RETURN_STR(GATT_CONN_L2C_FAILURE)
        CASE_RETURN_STR(GATT_CONN_TIMEOUT)
        CASE_RETURN_STR(GATT_CONN_TERMINATE_PEER_USER)
        CASE_RETURN_STR(GATT_CONN_TERMINATE_LOCAL_HOST)
        CASE_RETURN_STR(GATT_CONN_FAIL_ESTABLISH)
        CASE_RETURN_STR(GATT_CONN_LMP_TIMEOUT)
        CASE_RETURN_STR(GATT_CONN_CANCEL)
        CASE_RETURN_STR(HCI_ERR_PEER_POWER_OFF)

    }

    printf("Unknown Reason\r\n");
    return "UNKNOWN_REASON";
}

/**
 * Function Name:
 * get_gatt_status_name
 *
 * Function Description:
 * @brief  The function converts the wiced_bt_gatt_status_t enum value to its
 *        corresponding string literal. This will help the programmer to debug
 *        easily with log traces without navigating through the source code.
 *
 * @param status:   GATT status
 *
 * @return char*: String corresponding to the wiced_bt_gatt_status_t status
 */
const char* app_bt_util_get_gatt_status_name(wiced_bt_gatt_status_t status)
{

    switch ((int)status)
    {

        CASE_RETURN_STR(WICED_BT_GATT_SUCCESS || WICED_BT_GATT_ENCRYPTED_MITM)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_HANDLE)
        CASE_RETURN_STR(WICED_BT_GATT_READ_NOT_PERMIT)
        CASE_RETURN_STR(WICED_BT_GATT_WRITE_NOT_PERMIT)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_PDU)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_AUTHENTICATION)
        CASE_RETURN_STR(WICED_BT_GATT_REQ_NOT_SUPPORTED)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_OFFSET)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_AUTHORIZATION)
        CASE_RETURN_STR(WICED_BT_GATT_PREPARE_Q_FULL)
        CASE_RETURN_STR(WICED_BT_GATT_ATTRIBUTE_NOT_FOUND)
        CASE_RETURN_STR(WICED_BT_GATT_NOT_LONG)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_KEY_SIZE)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_ATTR_LEN)
        CASE_RETURN_STR(WICED_BT_GATT_ERR_UNLIKELY)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_ENCRYPTION)
        CASE_RETURN_STR(WICED_BT_GATT_UNSUPPORT_GRP_TYPE)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_RESOURCE)
        CASE_RETURN_STR(WICED_BT_GATT_ILLEGAL_PARAMETER)
        CASE_RETURN_STR(WICED_BT_GATT_NO_RESOURCES)
        CASE_RETURN_STR(WICED_BT_GATT_INTERNAL_ERROR)
        CASE_RETURN_STR(WICED_BT_GATT_WRONG_STATE)
        CASE_RETURN_STR(WICED_BT_GATT_DB_FULL)
        CASE_RETURN_STR(WICED_BT_GATT_BUSY)
        CASE_RETURN_STR(WICED_BT_GATT_ERROR)
        CASE_RETURN_STR(WICED_BT_GATT_CMD_STARTED)
        CASE_RETURN_STR(WICED_BT_GATT_PENDING)
        CASE_RETURN_STR(WICED_BT_GATT_AUTH_FAIL)
        CASE_RETURN_STR(WICED_BT_GATT_MORE)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_CFG)
        CASE_RETURN_STR(WICED_BT_GATT_SERVICE_STARTED)
        CASE_RETURN_STR(WICED_BT_GATT_ENCRYPTED_NO_MITM)
        CASE_RETURN_STR(WICED_BT_GATT_NOT_ENCRYPTED)
        CASE_RETURN_STR(WICED_BT_GATT_CONGESTED)
        CASE_RETURN_STR(WICED_BT_GATT_WRITE_REQ_REJECTED)
        CASE_RETURN_STR(WICED_BT_GATT_CCC_CFG_ERR)
        CASE_RETURN_STR(WICED_BT_GATT_PRC_IN_PROGRESS)
        CASE_RETURN_STR(WICED_BT_GATT_OUT_OF_RANGE)

    }

    return "UNKNOWN_STATUS";
}

/**
 * Function Name:
 * app_bt_util_get_pairing_status_name
 *
 * Function Description:
 * @brief Get the pairing status
 *
 * @param smp_status security manager status
 *
 * @return char*: String corresponding to the wiced_bt_smp_status_t smp_status
 */
const char* app_bt_util_get_pairing_status_name(wiced_bt_smp_status_t smp_status)
{

    switch ((int)smp_status)
    {
        CASE_RETURN_STR(SMP_SUCCESS)
        CASE_RETURN_STR(SMP_PASSKEY_ENTRY_FAIL)
        CASE_RETURN_STR(SMP_OOB_FAIL)
        CASE_RETURN_STR(SMP_PAIR_AUTH_FAIL)
        CASE_RETURN_STR(SMP_CONFIRM_VALUE_ERR)
        CASE_RETURN_STR(SMP_PAIR_NOT_SUPPORT)
        CASE_RETURN_STR(SMP_ENC_KEY_SIZE)
        CASE_RETURN_STR(SMP_INVALID_CMD)
        CASE_RETURN_STR(SMP_PAIR_FAIL_UNKNOWN)
        CASE_RETURN_STR(SMP_REPEATED_ATTEMPTS)
        CASE_RETURN_STR(SMP_INVALID_PARAMETERS)
        CASE_RETURN_STR(SMP_DHKEY_CHK_FAIL)
        CASE_RETURN_STR(SMP_NUMERIC_COMPAR_FAIL)
        CASE_RETURN_STR(SMP_BR_PAIRING_IN_PROGR)
        CASE_RETURN_STR(SMP_XTRANS_DERIVE_NOT_ALLOW)
        CASE_RETURN_STR(SMP_PAIR_INTERNAL_ERR)
        CASE_RETURN_STR(SMP_UNKNOWN_IO_CAP)
        CASE_RETURN_STR(SMP_INIT_FAIL)
        CASE_RETURN_STR(SMP_CONFIRM_FAIL)
        CASE_RETURN_STR(SMP_BUSY)
        CASE_RETURN_STR(SMP_ENC_FAIL)
        CASE_RETURN_STR(SMP_STARTED)
        CASE_RETURN_STR(SMP_RSP_TIMEOUT)
        CASE_RETURN_STR(SMP_FAIL)
        CASE_RETURN_STR(SMP_CONN_TOUT)

    }
    return "UNKNOWN_STATUS";
}

/**
 * Function Name:
 * app_bt_util_print_bd_address
 *
 * Function Description:
 * @brief This utility function prints the address of the Bluetooth device
 *
 * @param bdadr:  Bluetooth address
 *
 * @return void
 *
 */
void app_bt_util_print_bd_address(const wiced_bt_device_address_t bdadr)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X\r\n", bdadr[0],
                                              bdadr[1],
                                              bdadr[2],
                                              bdadr[3],
                                              bdadr[4],
                                              bdadr[5]);
}

/**
 * Function Name:
 * app_bt_util_print_byte_array
 *
 * Function Description:
 * @brief This is a utility function that prints the specified number of values
 * from memory
 *
 * @param to_print: Address of the location to print from
 * @param len: Number of bytes to be printed from the starting address
 *
 * @return void
 */
void app_bt_util_print_byte_array(void *to_print, uint16_t len)
{
    uint16_t counter;

    for (counter = 0; counter < len; counter++)
    {
        if (counter % 16 == 0)
        {
            printf("\r\n");
        }
        printf("%02X ", *(((uint8_t *)(to_print)) + counter));
    }
    printf("\r\n");

}

/**
 * Function Name:
 * app_bt_util_print_local_identity_key
 *
 * Function Description:
 * @brief Prints the given identity key
 *
 * @param st: Address of the location to print from
 * @param p_identity_key: pointer to identity key
 *
 * @return void
 */
void app_bt_util_print_local_identity_key(char *st, wiced_bt_local_identity_keys_t *p_identity_key)
{
    printf("%s local identity key\n", st);

    if(p_identity_key->key_type_mask == BTM_BLE_KEY_TYPE_ID)
    {
        printf("key type: %d (Identity resolving key)\n", p_identity_key->key_type_mask);
    }
    else if(p_identity_key->key_type_mask == BTM_BLE_KEY_TYPE_ER)
    {
        printf("key type: %d (Encryption root key)\n", p_identity_key->key_type_mask);
    }
    else
    {
        printf("key type: %d (unknown)\n", p_identity_key->key_type_mask);
    }

    printf("   ir: ");
    app_bt_util_print_byte_array(p_identity_key->id_keys.ir, LINK_KEY_LEN);
    printf("  irk: ");
    app_bt_util_print_byte_array(p_identity_key->id_keys.irk, LINK_KEY_LEN);
    printf("  dhk: ");
    app_bt_util_print_byte_array(p_identity_key->id_keys.dhk, LINK_KEY_LEN);
    printf("   er: ");
    app_bt_util_print_byte_array(p_identity_key->er, LINK_KEY_LEN);
}

/**
 * Function Name:
 * app_bt_util_print_link_key_data
 *
 * Function Description:
 * @brief Prints the given link key data
 *
 * @param p_key: pointer to link key
 *
 * @return void
 */
void app_bt_util_print_link_key_data(wiced_bt_device_sec_keys_t *p_key)
{
    printf("key mask: %02X\n", p_key->le_keys_available_mask);
    printf("add type: %d\n", p_key->ble_addr_type);
    printf("     irk: ");
    app_bt_util_print_byte_array(p_key->le_keys.irk, LINK_KEY_LEN);
    printf("    pltk: ");
    app_bt_util_print_byte_array(p_key->le_keys.pltk, LINK_KEY_LEN);
    printf("   pcsrk: ");
    app_bt_util_print_byte_array(p_key->le_keys.pcsrk, LINK_KEY_LEN);
    printf("    lltk: ");
    app_bt_util_print_byte_array(p_key->le_keys.lltk, LINK_KEY_LEN);
    printf("   lcsrk: ");
    app_bt_util_print_byte_array(p_key->le_keys.lcsrk, LINK_KEY_LEN);
}

/**
 * Function Name:
 * app_bt_util_generate_bd_address
 *
 * Function Description:
 * @brief Function used to generate unique BD Address for the device
 *
 * @param device_addr: pointer to read device address structure
 *
 * @return void
 */
void app_bt_util_generate_bd_address(const uint8_t *device_addr)
{
    printf("BD Address 0x%x%x%x%x%x%x%x%x \r\n",
            device_addr[0], device_addr[1],
            device_addr[2], device_addr[3],
            device_addr[4], device_addr[5],
            device_addr[6], device_addr[7]);
}
/* [] END OF FILE */
