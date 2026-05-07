#pragma once

#include "esp_err.h"
#include "lock_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Zigbee subsystem and start Zigbee + motor-command tasks.
 *        Must be called before esp_console_start_repl().
 *
 * @param motor  Pointer to the already-initialized lock motor instance.
 */
esp_err_t zigbee_init(lock_motor_t *motor);

/**
 * @brief Schedule a ZCL LockState attribute update in the Zigbee task context.
 *        Thread-safe: may be called from any task (console REPL, motor task, …).
 *
 * @param state  Current lock motor state.
 */
void zigbee_schedule_lock_state_update(lock_state_t state);

/**
 * @brief Schedule a Zigbee factory reset (clears network credentials, triggers re-pairing).
 *        Thread-safe: the reset is executed inside the Zigbee task context.
 */
void zigbee_factory_reset(void);

#ifdef __cplusplus
}
#endif
