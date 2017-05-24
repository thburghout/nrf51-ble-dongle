#include <stdint.h>
#include <nrf_delay.h>
#include <timer_issue.h>
#include <nrf_gpio.h>
#include <ble_advertising.h>
#include "softdevice_handler.h"
#include "app_timer.h"

#define APP_TIMER_OP_QUEUE_SIZE         4
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(175, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(195, UNIT_1_25_MS)
#define SLAVE_LATENCY                   2
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)
#define DEAD_BEEF                       0xDEADBEEF
#define APP_ADV_INTERVAL_FAST           320
#define APP_ADV_TIMEOUT_IN_SECONDS_FAST 0x3FFF
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

static void loop(void);
static void on_ble_evt(ble_evt_t *p_ble_evt);
static void ble_evt_dispatch(ble_evt_t *p_ble_evt);
void ble_stack_init(void);
void gap_params_init();
void advertising_init();
void on_adv_evt(const ble_adv_evt_t ble_adv_evt);
void advertising_start(void);


/**@brief Application main function.
 */
int main(void)
{
    // Initialize.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    ble_stack_init();
    gap_params_init();
    test_timer_init();
    advertising_init();
    advertising_start();

    // Enter main loop.
    for (;;)
    { loop(); }
}

static void loop(void)
{
}

///////// The BLE stack //////////

static void on_ble_evt(ble_evt_t *p_ble_evt)
{
    uint32_t err = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err           = sd_ble_gatts_sys_attr_set(m_conn_handle,
                                                      NULL,
                                                      0,
                                                      0);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err = sd_ble_gap_sec_params_reply(m_conn_handle,
                                              BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                              NULL,
                                              NULL);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            break;

        default:
            // No implementation needed.
            break;
    }
    APP_ERROR_CHECK(err);
}

static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    test_timer_on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    ble_advertising_on_sys_evt(sys_evt);
}

void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(
            NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_4000MS_CALIBRATION,
            NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params
                     .service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

}

void gap_params_init()
{
    uint32_t err_code;

    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&sec_mode);

    const uint8_t name[] = "12345679";
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          name,
                                          8);
    APP_ERROR_CHECK(err_code);

    ble_gap_conn_params_t conn_params = {0};
    conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    conn_params.slave_latency     = SLAVE_LATENCY;
    conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&conn_params);
    APP_ERROR_CHECK(err_code);
}

//////// Advertising ////////

void advertising_init()
{
    uint32_t               err_code;
    ble_advdata_t          advdata = {0};
    ble_adv_modes_config_t options = {0};

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = 0;

    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL_FAST;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS_FAST;
    options.ble_adv_slow_enabled  = BLE_ADV_SLOW_DISABLED;

    err_code = ble_advertising_init(&advdata,
                                    NULL,
                                    &options,
                                    on_adv_evt,
                                    NULL);
    APP_ERROR_CHECK(err_code);
}

void on_adv_evt(const ble_adv_evt_t ble_adv_evt)
{
    uint32_t err = NRF_SUCCESS;
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_IDLE:
            err = ble_advertising_start(BLE_ADV_MODE_SLOW);
            break;
        case BLE_ADV_EVT_FAST:
        case BLE_ADV_EVT_SLOW:
        default:
            break;
    }
    APP_ERROR_CHECK(err);
}

void advertising_start(void)
{
    uint32_t err = ble_advertising_start(BLE_ADV_MODE_FAST);
    if (err != NRF_SUCCESS && err != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err);
    }
}

/////// Error handlers //////


void app_error_handler(uint32_t error_code,
                       uint32_t line_num,
                       const uint8_t *p_file_name)
{
//    __BKPT(0);
    for (;;)
    {
        nrf_gpio_pin_toggle(PIN_DEBUG_1);
        nrf_delay_ms(50);
    }
//    NVIC_SystemReset();
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}
