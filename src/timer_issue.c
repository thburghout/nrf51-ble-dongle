#include <app_timer.h>
#include <sdk_common.h>
#include <nrf_gpio.h>
#include "timer_issue.h"


#define SECOND_TIMER_DELAY   APP_TIMER_TICKS(15000, APP_TIMER_PRESCALER)

APP_TIMER_DEF(m_inverted_timer);
APP_TIMER_DEF(m_second_timer);

void test_timer_on_ble_evt(ble_evt_t *p_ble_evt)
{
    uint32_t err = NRF_SUCCESS;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            stop_test_inverted_timer(); // removing this line completely
                                        // resolves the issue.
            NRF_TIMER2->TASKS_CLEAR = 		1;
            NRF_TIMER2->TASKS_START = 		1;
            err = app_timer_start(m_second_timer, APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER), NULL);
            nrf_gpio_pin_set(PIN_DEBUG_1);
            nrf_gpio_pin_set(PIN_DEBUG_2);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(PIN_DEBUG_1);
            nrf_gpio_pin_clear(PIN_DEBUG_2);
            err = app_timer_start(m_inverted_timer, SECOND_TIMER_DELAY, NULL);
            break;

        default:
            break;
    }
    APP_ERROR_CHECK(err);
}

void stop_test_inverted_timer()
{
    uint32_t err = app_timer_stop(m_inverted_timer);
    if (err != NRF_SUCCESS && err != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err);
    }
}


void TIMER2_IRQHandler(void)
{
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    nrf_gpio_pin_clear(PIN_DEBUG_1);
}

void on_test_timeout(void *p_context)
{
    // This handler can be delayed to more than 1 minute.
    UNUSED_PARAMETER(p_context);
    nrf_gpio_pin_clear(PIN_DEBUG_2);
}

void test_timer_init()
{
    nrf_gpio_cfg_output(PIN_DEBUG_1);
    nrf_gpio_cfg_output(PIN_DEBUG_2);
    nrf_gpio_pin_clear(PIN_DEBUG_1);
    nrf_gpio_pin_clear(PIN_DEBUG_2);

    uint32_t err = app_timer_create(&m_inverted_timer,
                                    APP_TIMER_MODE_REPEATED,
                                    test_timer_callback);
    APP_ERROR_CHECK(err);
    err = app_timer_create(&m_second_timer,
                                    APP_TIMER_MODE_SINGLE_SHOT,
                                    on_test_timeout);
    APP_ERROR_CHECK(err);

    NRF_TIMER2->TASKS_STOP = 		1;
    NRF_TIMER2->TASKS_CLEAR = 		1;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    NRF_TIMER2->EVENTS_COMPARE[1] = 0;
    NRF_TIMER2->EVENTS_COMPARE[2] = 0;
    NRF_TIMER2->EVENTS_COMPARE[3] = 0;
    NRF_TIMER2->CC[0] = 			62500;
    NRF_TIMER2->MODE =				(TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
    NRF_TIMER2->PRESCALER =			(9 << TIMER_PRESCALER_PRESCALER_Pos);
    NRF_TIMER2->BITMODE =			(TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos);
    NRF_TIMER2->SHORTS =			(TIMER_SHORTS_COMPARE0_STOP_Enabled << TIMER_SHORTS_COMPARE0_STOP_Pos);

    NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    NVIC_SetPriority(TIMER2_IRQn, 3);
    NVIC_EnableIRQ(TIMER2_IRQn);

    err = app_timer_start(m_second_timer, APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err);
}

void test_timer_callback(void *p_context)
{
    UNUSED_PARAMETER(p_context);
}