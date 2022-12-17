#ifndef _GLOBALS_H_
#define _GLOBALS_H_

//Standard Library Includes
#include <stdint.h>
#include <string.h>

//Toolchain Includes
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

//Logging Includes
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//Definitions
#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /* Is on when device is advertising                                                                   */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /* Is on when device has connected                                                                    */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /* LED to be toggled with the help of the LED Button Service                                          */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /* Button that will trigger the notification event with the LED Button Service                        */

#define DEVICE_NAME                     "Nordic_Blinky"                         /* Name of device. Will be included in the advertising data                                           */

#define APP_BLE_OBSERVER_PRIO           3                                       /* Application's BLE observer priority. You shouldn't need to modify this value                       */
#define APP_BLE_CONN_CFG_TAG            1                                       /* A tag identifying the SoftDevice BLE configuration                                                 */

#define APP_ADV_INTERVAL                64                                      /* The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms)                   */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /* The advertising time-out (in units of seconds). When set to 0, we will never time out              */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /* Minimum acceptable connection interval (0.5 seconds)                                               */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /* Maximum acceptable connection interval (1 second)                                                  */
#define SLAVE_LATENCY                   0                                       /* Slave latency                                                                                      */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /* Connection supervisory time-out (4 seconds)                                                        */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /* Time from initiating event (connect or start of notification) to first time                        */
                                                                                /* sd_ble_gap_conn_param_update is called (15 seconds)                                                */  
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /* Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds)            */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /* Number of attempts before giving up the connection parameter negotiation                           */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /* Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks)          */

#define DEAD_BEEF                       0xDEADBEEF                              /* Value used as error code on stack dump, can be used to identify stack location on stack unwind     */


BLE_LBS_DEF(m_lbs);                                                             /* LED Button Service instance                                                                        */
NRF_BLE_GATT_DEF(m_gatt);                                                       /* GATT module instance                                                                               */
NRF_BLE_QWR_DEF(m_qwr);                                                         /* Context for the Queued Write module                                                                */

#endif /* _GLOBALS_H_ */

