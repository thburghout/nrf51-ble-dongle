#ifndef CABLE_DETECTION_H
#define CABLE_DETECTION_H

#include <ble.h>
#include "nrf_delay.h"

#define APP_TIMER_PRESCALER             0

#define PIN_DEBUG_1							17
#define PIN_DEBUG_2							15
//#define PIN_DEBUG_1							22
//#define PIN_DEBUG_2							21

/**@brief   Initializes this module by creating and enabling the timer.
 */
void test_timer_init();

/**@brief   Handles BLE events, is expected to be called on every BLE event for
 *          this module to function properly.
 *
 * @param   p_ble_evt   The fired BLE event.
 */
void test_timer_on_ble_evt(ble_evt_t *p_ble_evt);

/**@brief   Stops the timer.
 *
 * @note    Ignores a NRF_ERROR_INVALID_STATE error, mostly fired when stopping
 *          a timer which isn't running.
 */
static void stop_test_inverted_timer();

/**@brief
 *
 * @param   p_context   Unused parameter.
 */
static void test_timer_callback(void *p_context);

#endif // CABLE_DETECTION_H
