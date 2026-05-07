#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize TTP223 touch button inputs and start the handler task.
 *        GPIO CONFIG_LOCKBEE_TOUCH_OPEN_GPIO  → open (unlock)
 *        GPIO CONFIG_LOCKBEE_TOUCH_CLOSE_GPIO → close (lock)
 *
 *        Must be called after zigbee_init() (uses zigbee_motor_cmd_send).
 */
esp_err_t touch_init(void);

#ifdef __cplusplus
}
#endif
