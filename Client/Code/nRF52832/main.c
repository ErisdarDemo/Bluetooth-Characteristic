/*************************************************************************************************/
/** @file	main.c
 *  @brief	BLE LED Button Service central and client application main file.
 *  @details	Source code for a sample client application using the LED Button service.
 *
 *  @author	Justin Reina, Firmware Engineer
 *  @source     examples\ble_central\ble_app_blinky_c
 *  @created	12/16/22
 *  @last rev	12/16/22
 *
 *
 *  @notes	x
 *
 *  @section	Opens
 *      none current
 *
 *  @section	Legal Disclaimer
 *      © 2022 Justin Reina, All rights reserved. All contents of this source file and/or 
 *      any other related source  files are the explicit property of Justin Reina. 
 *      Do not distribute. Do not copy.
 */
/*************************************************************************************************/

//Standard Library Includes
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//Toolchain Includes
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"

//Logging Includes
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//Definitions
#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /* Scanning LED will be on when the device is scanning                                                    */
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1                     /* Connected LED will be on when the device is connected                                                  */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                     /* LED to indicate a change of state of the the Button characteristic on the peer                         */

#define SCAN_INTERVAL                   0x00A0                              /* Determines scan interval in units of 0.625 millisecond                                                 */
#define SCAN_WINDOW                     0x0050                              /* Determines scan window in units of 0.625 millisecond                                                   */
#define SCAN_DURATION                   0x0000                              /* Timout when scanning. 0x0000 disables timeout                                                          */

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /* Determines minimum connection interval in milliseconds                                                 */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /* Determines maximum connection interval in milliseconds                                                 */
#define SLAVE_LATENCY                   0                                   /* Determines slave latency in terms of connection events                                                 */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /* Determines supervision time-out in units of 10 milliseconds                                            */

#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                        /* Button that will write to the LED characteristic of the peer                                           */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                 /* Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks)              */

#define APP_BLE_CONN_CFG_TAG            1                                   /* A tag identifying the SoftDevice BLE configuration                                                     */
#define APP_BLE_OBSERVER_PRIO           3                                   /* Application's BLE observer priority. You shouldn't need to modify this value                           */

NRF_BLE_SCAN_DEF(m_scan);                                                   /* Scanning module instance                                                                               */
BLE_LBS_C_DEF(m_ble_lbs_c);                                                 /* Main structure used by the LBS client module                                                           */
NRF_BLE_GATT_DEF(m_gatt);                                                   /* GATT module instance                                                                                   */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                            /* DB discovery module instance                                                                           */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                            /* BLE GATT Queue instance                                                                                */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

//Local Constants
static char const m_target_periph_name[] = "Nordic_Blinky";                 /* Name of the device we try to connect to. This name is searched in the scan report data                 */


/*************************************************************************************************/
/** @fcn	void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
 *  @brief	function to handle asserts in the SoftDevice
 *  @details	this function will be called in case of an assert in the SoftDevice
 *
 *
 *  @param      [in] (uint16_t) line_num - Line number of the failing ASSERT call
 *  @param      [in] (const uint8_t *) p_file_name - File name of the failing ASSERT call
 *
 *  @warn   On assert from the SoftDevice, the system can only recover on reset.
 */
/*************************************************************************************************/
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {

    app_error_handler(0xDEADBEEF, line_num, p_file_name);

    return;
}


/*************************************************************************************************/
/** @fcn	static void lbs_error_handler(uint32_t nrf_error)
 *  @brief	function for handling the LED Button Service client errors
 *  @details	x
 *
 *  @param	[in] (uint32_t) nrf_error - Error code containing information about what went wrong
 */
/*************************************************************************************************/
static void lbs_error_handler(uint32_t nrf_error) {

    APP_ERROR_HANDLER(nrf_error);

    return;
}


/*************************************************************************************************/
/** @fcn	static void leds_init(void)
 *  @brief	function for the LEDs initialization
 *  @details	initializes all LEDs used by the application
 */
/*************************************************************************************************/
static void leds_init(void) {

    bsp_board_init(BSP_INIT_LEDS);

    return;
}


/*************************************************************************************************/
/** @fcn	static void scan_start(void)
 *  @brief	function to start scanning
 *  @details	x
 */
/*************************************************************************************************/
static void scan_start(void) {

    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_off(CENTRAL_CONNECTED_LED);
    bsp_board_led_on(CENTRAL_SCANNING_LED);

    return;
}


/*************************************************************************************************/
/** @fcn	static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
 *  @brief	handles events coming from the LED Button central module
 *  @details	x
*
 *  @param	[in] (ble_lbs_c_t *) p_lbs_c - x
 *  @param	[in] (ble_lbs_c_evt_t *) p_lbs_c_evt - x
 */
/*************************************************************************************************/
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt) {

    switch (p_lbs_c_evt->evt_type) {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE: {
            ret_code_t err_code;

            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c,
                                                p_lbs_c_evt->conn_handle,
                                                &p_lbs_c_evt->params.peer_db);
            NRF_LOG_INFO("LED Button service discovered on conn_handle 0x%x.", p_lbs_c_evt->conn_handle);

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION: {
            NRF_LOG_INFO("Button state changed on peer to 0x%x.", p_lbs_c_evt->params.button.button_state);
            if (p_lbs_c_evt->params.button.button_state) {
                bsp_board_led_on(LEDBUTTON_LED);
            } else {
                bsp_board_led_off(LEDBUTTON_LED);
            }
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }

    return;
}


/*************************************************************************************************/
/** @fcn	static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
 *  @brief	function for handling BLE events
 *  @details	x
 *
 *  @param      [in] (ble_evt_t const *) p_ble_evt - Bluetooth stack event
 *  @param      [in] ( void *) p_context - Unused
 */
/*************************************************************************************************/
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {

    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id) {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED: {
            NRF_LOG_INFO("Connected.");
            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c, p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            // Update LEDs status, and check if we should be looking for more periphs to connect to.
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            bsp_board_led_off(CENTRAL_SCANNING_LED);
        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED: {
            NRF_LOG_INFO("Disconnected.");
            scan_start();
        } break;

        case BLE_GAP_EVT_TIMEOUT: {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT: {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT: {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }

    return;
}


/*************************************************************************************************/
/** @fcn	static void lbs_c_init(void)
 *  @brief	LED Button client initialization
 *  @details	x
 */
/*************************************************************************************************/
static void lbs_c_init(void) {

    ret_code_t       err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler   = lbs_c_evt_handler;
    lbs_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;
    lbs_c_init_obj.error_handler = lbs_error_handler;

    err_code = ble_lbs_c_init(&m_ble_lbs_c, &lbs_c_init_obj);
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	static void ble_stack_init(void)
 *  @brief	function for initializing the BLE stack
 *  @details	initializes the SoftDevice and the BLE event interrupts
 */
/*************************************************************************************************/
static void ble_stack_init(void) {

    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    return;
}


/*************************************************************************************************/
/** @fcn	static void button_event_handler(uint8_t pin_no, uint8_t button_action)
 *  @brief	function for handling events from the button handler module
 *  @details	x
 *
 *  @param      [in] (uint8_t) pin_no - The pin that the event applies to.
 *  @param      [in] (uint8_t) button_action - The button action (press/release).
 */
/*************************************************************************************************/
static void button_event_handler(uint8_t pin_no, uint8_t button_action) {

    ret_code_t err_code;

    switch (pin_no) {

        case LEDBUTTON_BUTTON_PIN:
            err_code = ble_lbs_led_status_send(&m_ble_lbs_c, button_action);
            if ((err_code != NRF_SUCCESS) && (err_code != BLE_ERROR_INVALID_CONN_HANDLE) && (err_code != NRF_ERROR_INVALID_STATE)) {
                APP_ERROR_CHECK(err_code);
            }

            if (err_code == NRF_SUCCESS) {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }

    return;
}


/*************************************************************************************************/
/** @fcn	static void scan_evt_handler(scan_evt_t const * p_scan_evt)
 *  @brief	function for handling Scaning events
 *  @details	x
 *
 *  @param	[in] ((scan_evt_t const *) p_scan_evt - Scanning event
 */
/*************************************************************************************************/
static void scan_evt_handler(scan_evt_t const * p_scan_evt) {

    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id) {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
            break;
        default:
          break;
    }

    return;
}


/*************************************************************************************************/
/** @fcn	static void buttons_init(void)
 *  @brief	function for initializing the button handler modul
 *  @details	x
 */
/*************************************************************************************************/
static void buttons_init(void) {

    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
 *  @brief	function for handling database discovery events
 *  @details	This function is callback function to handle events from the database discovery 
 *              module. Depending on the UUIDs that are discovered, this function should forward 
 *              the events to their respective services
 *
 *  @param	[in] (ble_db_discovery_evt_t *) p_event - Pointer to the database discovery event
 */
/*************************************************************************************************/
static void db_disc_handler(ble_db_discovery_evt_t * p_evt) {

    ble_lbs_on_db_disc_evt(&m_ble_lbs_c, p_evt);

    return;
}


/*************************************************************************************************/
/** @fcn	static void db_discovery_init(void)
 *  @brief	database discovery initialization
 *  @details	x
 */
/*************************************************************************************************/
static void db_discovery_init(void) {

    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	static void log_init(void)
 *  @brief	function for initializing the log
 *  @details	x
 */
/*************************************************************************************************/
static void log_init(void) {

    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    return;
}


/*************************************************************************************************/
/** @fcn	static void timer_init(void)
 *  @brief	function for initializing the timer
 *  @details	x
 */
/*************************************************************************************************/
static void timer_init(void) {

    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	static void power_management_init(void)
 *  @brief	function for initializing the Power manager
 *  @details	x
 */
/*************************************************************************************************/
static void power_management_init(void) {

    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	static void scan_init(void)
 *  @brief	x
 *  @details	x
 */
/*************************************************************************************************/
static void scan_init(void) {

    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    // Setting filters for scanning.
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	static void gatt_init(void)
 *  @brief	function for initializing the GATT module
 *  @details	x
 */
/*************************************************************************************************/
static void gatt_init(void) {

    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	static void idle_state_handle(void)
 *  @brief	function for handling the idle state (main loop)
 *  @details	handle any pending log operation(s), then sleep until the next event occurs
 */
/*************************************************************************************************/
static void idle_state_handle(void) {

    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();

    return;
}


/*************************************************************************************************/
/** @fcn	int main(void)
 *  @brief	function for application main entry
 *  @details	x
 *		
 *  @section	Opens
 *     stdComments
 *     printf on activities
 *     activities for all buttons
 *     ...!
 */
/*************************************************************************************************/
int main(void) {

    // Initialize.
    log_init();
    timer_init();
    leds_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    scan_init();
    gatt_init();
    db_discovery_init();
    lbs_c_init();

    // Start execution.
    NRF_LOG_INFO("Blinky CENTRAL example started.");
    scan_start();

    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);

    // Enter main loop.
    for (;;) {
        idle_state_handle();
    }
}

