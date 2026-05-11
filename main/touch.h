#pragma once

#include "esp_err.h"
#include "lock_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize TTP223 touch button inputs and start the handler task.
 *        GPIO CONFIG_LOCKBEE_TOUCH_OPEN_GPIO  → open (unlock)
 *        GPIO CONFIG_LOCKBEE_TOUCH_CLOSE_GPIO → close (lock)
 *
 *        Must be called after zigbee_init() (uses zigbee_motor_cmd_send).
 *
 * @param motor  Pointer to the initialized motor handle (used to check busy state).
 */
esp_err_t touch_init(lock_motor_t *motor);

/**
 * @brief Enable or disable touch button actions.
 *        When disabled, button presses are silently ignored.
 *        Controlled via ZCL Door Lock attribute EnableOneTouchLocking (0x0029).
 */
void touch_set_enabled(bool enabled);
bool touch_get_enabled(void);

#ifdef __cplusplus
}
#endif
