/*************************************************************************************************/
/** @file	main.c
 *  @brief	Blinky Sample Application main file.
 *  @details	Source code for a sample server application using the LED Button service
 *
 *  @author	Justin Reina, Firmware Engineer
 *  @source     examples\ble_peripheral\ble_app_blinky
 *  @created	12/16/22
 *  @last rev	12/18/22
 *
 *
 *  @section	Opens
 *      extract globals.h:52-58 to undeffed, non-static declarations
 *      btlib.c/h
 *      system.c/h
 *      tie all (4) LED/Button pairs (sep bt-chars?)
 *      break into demo form
 *       flush out distracting content to bt_lib.c/h
 *       establish target demo form
 *      printf on activities
 *      activities for all buttons
 *      merge to single source file set!
 *      ...!
 *
 *  @section	Legal Disclaimer
 *      © 2022 Justin Reina, All rights reserved. All contents of this source file and/or 
 *      any other related source  files are the explicit property of Justin Reina. 
 *      Do not distribute. Do not copy.
 */
/*************************************************************************************************/
#include "globals.h"


//Local Constants
uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /* Handle of the current connection                                                                   */

uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /* Advertising handle used to identify an advertising set                                             */
uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /* Buffer for storing an encoded advertising set                                                      */
uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /* Buffer for storing an encoded scan data                                                            */

/**@brief Struct that contains pointers to the encoded advertising data. */
ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};


/*************************************************************************************************/
/** @fcn	void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
 *  @brief	function for assert macro callback
 *  @details	will be called in case of an assert in the SoftDevice.
 *
 *  @param      [in] (uint16_t) line_num - Line number of the failing ASSERT call
 *  @param      [in] (const uint8_t *) p_file_name - File name of the failing ASSERT call
 *
 *  @warn     On assert from the SoftDevice, the system can only recover on reset.
 */
/*************************************************************************************************/
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {

    app_error_handler(DEAD_BEEF, line_num, p_file_name);

    return;
}


/*************************************************************************************************/
/** @fcn	void leds_init(void)
 *  @brief	function for the LEDs initialization
 *  @details	initializes all LEDs used by the application
 */
/*************************************************************************************************/
void leds_init(void) {

    bsp_board_init(BSP_INIT_LEDS);

    return;
}


/*************************************************************************************************/
/** @fcn	void timers_init(void)
 *  @brief	function for the Timer initialization
 *  @details	initializes the timer module
 */
/*************************************************************************************************/
void timers_init(void) {

    //Locals
    ret_code_t err_code;
    
    //Init
    err_code = app_timer_init();                  /* Init timer module, use the scheduler        */

    
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	void gap_params_init(void)
 *  @brief	function for the GAP initialization
 *  @details	Sets up all necessary GAP params of the device including the device name, 
 *              appearance, and the preferred connection parameters.
 */
/*************************************************************************************************/
void gap_params_init(void) {

    //Locals
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;


    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	void gatt_init(void)
 *  @brief	function for initializing the GATT module
 *  @details	x
 */
/*************************************************************************************************/
void gatt_init(void) {

    //Locals
    ret_code_t err_code;

    
    //Init
    err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	void advertising_init(void)
 *  @brief	function for initializing the Advertising functionality
 *  @details	encodes the required advertising data and passes it to the stack. Also builds a 
 *              structure to be passed to the stack when starting advertising.
 */
/*************************************************************************************************/
void advertising_init(void) {

    //Locals
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};


    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	void nrf_qwr_error_handler(uint32_t nrf_error)
 *  @brief	function for handling Queued Write Module errors
 *  @details	a pointer to this function will be passed to each service which may need to 
 *              inform the application about an error.
 *
 *  @param	[in] (uint32_t) nrf_error - Error code containing information about what went wrong.
 */
/*************************************************************************************************/
void nrf_qwr_error_handler(uint32_t nrf_error) {

    APP_ERROR_HANDLER(nrf_error);

    return;
}


/*************************************************************************************************/
/** @fcn	void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
 *  @brief	function for handling write events to the LED characteristic
 *  @details	x
 *
 *  @param	[in] (uint16_t) conn_handle - x
 *  @param      [in] (ble_lbs_t *) p_lbs - Instance of LED Button Service to which the write applies.
 *  @param      [in] (uint8_t) led_state - Written/desired state of the LED.
 */
/*************************************************************************************************/
void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state) {

    if (led_state) {
        bsp_board_led_on(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED ON!");
    } else {
        bsp_board_led_off(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED OFF!");
    }

    return;
}


/*************************************************************************************************/
/** @fcn	void services_init(void)
 *  @brief	function for initializing services that will be used by the application
 *  @details	x
 */
/*************************************************************************************************/
void services_init(void) {

    //Locals
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    
    APP_ERROR_CHECK(err_code);

    // Initialize LBS.
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
 *  @brief	function for handling the Connection Parameters Module
 *  @details	this function will be called for all events in the Connection Parameters Module 
 *              that are passed to the application
 *
 *  @param	[in] (ble_conn_params_evt_t *) p_evt - Event received from the Connection 
                                                       Parameters Module
 *
 *  @section	Note
 *  	All this function does is to disconnect. This could have been done by simply setting the 
 *      disconnect_on_fail config parameter, but instead we use the event handler mechanism to 
 *      demonstrate its use.
 */
/*************************************************************************************************/
void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {

    //Locals
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        
        APP_ERROR_CHECK(err_code);
    }

    return;
}


/*************************************************************************************************/
/** @fcn	void conn_params_error_handler(uint32_t nrf_error)
 *  @brief	function for handling a Connection Parameters error
 *  @details	x
 *
 *  @param	[in] (uint32_t)  nrf_error - Error code containing information about what went wrong
 */
/*************************************************************************************************/
void conn_params_error_handler(uint32_t nrf_error) {

    APP_ERROR_HANDLER(nrf_error);

    return;
}


/*************************************************************************************************/
/** @fcn	void conn_params_init(void)
 *  @brief	function for initializing the Connection Parameters module
 *  @details	x
 */
/*************************************************************************************************/
void conn_params_init(void) {

    //Locals
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    //Init
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    
    APP_ERROR_CHECK(err_code);
    
    return;
}


/*************************************************************************************************/
/** @fcn	void advertising_start(void)
 *  @brief	function for starting advertising
 *  @details	x
 */
/*************************************************************************************************/
void advertising_start(void) {

    //Locasls
    ret_code_t err_code;


    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    
    APP_ERROR_CHECK(err_code);

    bsp_board_led_on(ADVERTISING_LED);

    return;
}


/*************************************************************************************************/
/** @fcn	void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
 *  @brief	function for handling BLE events
 *  @details	x
 *
 *  @param      [in] (ble_evt_t const *) p_ble_evt - Bluetooth stack event
 *  @param      [in] (void *) p_context - Unused
 */
/*************************************************************************************************/
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {

    //Locals
    ret_code_t err_code;


    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            bsp_board_led_on(CONNECTED_LED);
            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
           
            APP_ERROR_CHECK(err_code);
            
            err_code = app_button_enable();
           
            APP_ERROR_CHECK(err_code);
           
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            
            err_code = app_button_disable();
            
            APP_ERROR_CHECK(err_code);
            
            advertising_start();
           
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            NRF_LOG_DEBUG("PHY update request.");

            ble_gap_phys_t const phys = {.rx_phys = BLE_GAP_PHY_AUTO, .tx_phys = BLE_GAP_PHY_AUTO};
            
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);

            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
         
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
         
            APP_ERROR_CHECK(err_code);
         
            break;

        case BLE_GATTC_EVT_TIMEOUT:
          
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
          
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
          
            APP_ERROR_CHECK(err_code);
            
            break;

        case BLE_GATTS_EVT_TIMEOUT:

            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
           
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
           
            APP_ERROR_CHECK(err_code);
           
            break;

        default:
            break;                                /* no resp                                     */
    }

    return;
}


/*************************************************************************************************/
/** @fcn	void ble_stack_init(void)
 *  @brief	function for initializing the BLE stack
 *  @details	initializes the SoftDevice and the BLE event interrupt
 */
/*************************************************************************************************/
void ble_stack_init(void) {

    //Locals
    ret_code_t err_code;
    uint32_t ram_start;


    err_code = nrf_sdh_enable_request();
    
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    ram_start = 0;
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
/** @fcn	void button_event_handler(uint8_t pin_no, uint8_t button_action)
 *  @brief	function for handling events from the button handler module
 *  @details	x
 *
 *  @param      [in] (uint8_t) pin_no - pin that the event applies to
 *  @param      [in] (uint8_t) button_action - the button action (press/release)
 */
/*************************************************************************************************/
void button_event_handler(uint8_t pin_no, uint8_t button_action) {

    //Locals
    ret_code_t err_code;


    switch (pin_no) {
        case LEDBUTTON_BUTTON:
            NRF_LOG_INFO("Send button state change.");
            err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }

    return;
}


/*************************************************************************************************/
/** @fcn	void buttons_init(void)
 *  @brief	function for initializing the button handler module
 *  @details	x
 *
 *  @note   buttons are so a ptr to it will be saved in the button handler module
 */
/*************************************************************************************************/
void buttons_init(void) {

    //Locals
    ret_code_t err_code;

    app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	void log_init(void)
 *  @brief	x
 *  @details	x
 */
/*************************************************************************************************/
void log_init(void) {

    //Locals
    ret_code_t err_code;

    
    //Init
    err_code = NRF_LOG_INIT(NULL);
    
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    return;
}


/*************************************************************************************************/
/** @fcn	void power_management_init(void)
 *  @brief	function for initializing power management
 *  @details	x
 */
/*************************************************************************************************/
void power_management_init(void) {

    //Locals
    ret_code_t err_code;
    
    //Init
    err_code = nrf_pwr_mgmt_init();
   
    APP_ERROR_CHECK(err_code);

    return;
}


/*************************************************************************************************/
/** @fcn	void idle_state_handle(void)
 *  @brief	function for handling the idle state (main loop)
 *  @details	if there is no pending log operation, then sleep until next the next event occurs
 */
/*************************************************************************************************/
void idle_state_handle(void) {

    if (NRF_LOG_PROCESS() == false) {
        nrf_pwr_mgmt_run();
    }

    return;
}


/*************************************************************************************************/
/** @fcn	int main(void)
 *  @brief	function for application main entry
 *  @details	x
 */
/*************************************************************************************************/
int main(void) {

    // Initialize.
    log_init();
    leds_init();
    timers_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO("Blinky example started.");
    advertising_start();

    // Enter main loop.
    for (;;) {
        idle_state_handle();
    }
}

