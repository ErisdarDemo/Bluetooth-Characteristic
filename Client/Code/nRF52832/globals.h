#ifndef _GLOBALS_H_
#define _GLOBALS_H_

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


#endif /* _GLOBALS_H_ */

