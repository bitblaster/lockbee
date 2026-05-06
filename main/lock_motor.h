#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "tmc2209.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gptimer.h"

// ---------------------------------------------------------------------------
// Lock state
// ---------------------------------------------------------------------------
typedef enum {
    LOCK_STATE_UNKNOWN   = 0,
    LOCK_STATE_LOCKED,
    LOCK_STATE_UNLOCKED,
    LOCK_STATE_MOVING,
    LOCK_STATE_ERROR,
} lock_state_t;

// ---------------------------------------------------------------------------
// Configuration (pass at init, then stays constant)
// ---------------------------------------------------------------------------
typedef struct {
    // GPIO
    int step_gpio;
    int dir_gpio;
    int en_gpio;
    int diag_gpio;

    // Motor geometry
    int   steps_per_rev;      // full steps/rev (200 for NEMA17 1.8°)
    int   microsteps;         // microstepping divisor
    int   lock_steps;         // microsteps from locked to unlocked (0=auto-calc)
    int   lock_turns;         // full revolutions to open lock

    // Timing
    uint32_t step_period_us;  // period between steps in µs (speed control)
    uint32_t accel_steps;     // ramp-up steps (0 = no acceleration)

    // Current
    uint8_t irun;
    uint8_t ihold;
    uint8_t ihold_delay;

    // StallGuard
    uint8_t  sg_threshold;
    uint32_t sg_tcoolthrs;

    // TMC2209 UART
    int     tmc_uart_port;
    int     tmc_tx_gpio;
    int     tmc_rx_gpio;
    uint8_t tmc_slave_addr;
} lock_motor_config_t;

// ---------------------------------------------------------------------------
// Motor handle (internal state)
// ---------------------------------------------------------------------------
typedef struct {
    lock_motor_config_t cfg;
    tmc2209_t           tmc;

    // Step generation (GPTimer)
    gptimer_handle_t    timer;
    SemaphoreHandle_t   done_sem;

    // Volatile state (shared with ISR)
    volatile int32_t  position;         // current position in microsteps (signed)
    volatile int32_t  steps_remaining;  // countdown
    volatile bool     stall_detected;
    volatile bool     running;
    volatile bool     dir_fwd;          // current direction

    // Lock state
    lock_state_t      state;
    int32_t           locked_pos;    // position when fully locked
    int32_t           unlocked_pos;  // position when fully unlocked
} lock_motor_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

/**
 * @brief Initialize motor hardware: GPIO, GPTimer, TMC2209 UART, and register setup.
 */
esp_err_t lock_motor_init(lock_motor_t *motor, const lock_motor_config_t *cfg);

/**
 * @brief Enable or disable the TMC2209 (EN pin).
 *        Disable to reduce heat when idle for extended periods.
 */
esp_err_t lock_motor_enable(lock_motor_t *motor, bool enable);

/**
 * @brief Move a given number of microsteps asynchronously (returns immediately).
 *        Call lock_motor_wait() to block until complete.
 *
 * @param steps      Number of microsteps to travel
 * @param forward    Direction: true = forward, false = reverse
 */
esp_err_t lock_motor_move(lock_motor_t *motor, int32_t steps, bool forward);

/**
 * @brief Move and wait for completion or timeout.
 *
 * @param timeout_ms  0 = wait forever
 * @return ESP_OK on completion, ESP_ERR_TIMEOUT, or error
 */
esp_err_t lock_motor_move_wait(lock_motor_t *motor,
                                int32_t steps, bool forward,
                                uint32_t timeout_ms);

/**
 * @brief Wait for a previously started move to complete.
 */
esp_err_t lock_motor_wait(lock_motor_t *motor, uint32_t timeout_ms);

/**
 * @brief Stop the motor immediately.
 */
esp_err_t lock_motor_stop(lock_motor_t *motor);

/**
 * @brief Calibrate home position by moving until stall, then set position = 0.
 *        Direction: true = forward (toward locked end-stop).
 */
esp_err_t lock_motor_calibrate(lock_motor_t *motor, bool forward);

/**
 * @brief Move to the unlocked position (previously calibrated or calculated).
 */
esp_err_t lock_motor_open(lock_motor_t *motor);

/**
 * @brief Move to the locked position.
 */
esp_err_t lock_motor_close(lock_motor_t *motor);

/** @brief Query current lock state. */
lock_state_t lock_motor_get_state(lock_motor_t *motor);

/** @brief Query current position in microsteps. */
int32_t lock_motor_get_position(lock_motor_t *motor);

/** @brief Check if last move ended in a stall. */
bool lock_motor_is_stalled(lock_motor_t *motor);

/** @brief Set step period (speed) in µs between steps. */
void lock_motor_set_speed(lock_motor_t *motor, uint32_t step_period_us);

/** @brief Return lock state as a string (for logging). */
const char *lock_motor_state_str(lock_state_t state);
