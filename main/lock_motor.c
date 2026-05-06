#include "lock_motor.h"

#include <string.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "lock_motor";

// ---------------------------------------------------------------------------
// GPTimer ISR callback — called at step_period_us interval.
// Generates one STEP pulse per call and decrements the step counter.
// Returns true if a high-priority task was woken (for FreeRTOS scheduling).
// ---------------------------------------------------------------------------
static bool IRAM_ATTR step_timer_cb(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *arg)
{
    lock_motor_t *m = (lock_motor_t *)arg;
    BaseType_t woken = pdFALSE;

    if (!m->running) {
        return false;
    }

    // Check stall flag (set by DIAG GPIO ISR)
    if (m->stall_detected || m->steps_remaining <= 0) {
        m->running = false;
        xSemaphoreGiveFromISR(m->done_sem, &woken);
        return woken == pdTRUE;
    }

    // Generate STEP pulse (rising edge → falling edge)
    // TMC2209 requires tSH >= 100 ns; two GPIO writes ≫ 100 ns
    gpio_set_level(m->cfg.step_gpio, 1);
    esp_rom_delay_us(1);  // ensure minimum pulse width
    gpio_set_level(m->cfg.step_gpio, 0);

    // Update position
    m->steps_remaining--;
    if (m->dir_fwd) {
        m->position++;
    } else {
        m->position--;
    }

    // Signal completion when last step is done
    if (m->steps_remaining == 0) {
        m->running = false;
        xSemaphoreGiveFromISR(m->done_sem, &woken);
    }

    return woken == pdTRUE;
}

// ---------------------------------------------------------------------------
// DIAG GPIO ISR — fires on rising edge when TMC2209 detects a stall
// ---------------------------------------------------------------------------
static void IRAM_ATTR diag_isr_handler(void *arg)
{
    lock_motor_t *m = (lock_motor_t *)arg;
    m->stall_detected = true;
    // The step_timer_cb will see this flag and give the semaphore on the next tick
}

// ---------------------------------------------------------------------------
// Apply TMC2209 register configuration — called at init and after power-cycle
// ---------------------------------------------------------------------------
static esp_err_t apply_tmc2209_config(lock_motor_t *motor)
{
    // GCONF: enable PDN (UART mode), use register for MSTEP, StealthChop by default
    uint32_t gconf = TMC2209_GCONF_PDN_DISABLE
                   | TMC2209_GCONF_MSTEP_REG_SELECT
                   | TMC2209_GCONF_MULTISTEP_FILT;
    ESP_RETURN_ON_ERROR(tmc2209_write_reg(&motor->tmc, TMC2209_REG_GCONF, gconf),
                        TAG, "GCONF write failed");

    ESP_RETURN_ON_ERROR(tmc2209_set_current(&motor->tmc,
                                             motor->cfg.irun,
                                             motor->cfg.ihold,
                                             motor->cfg.ihold_delay),
                        TAG, "set_current failed");

    ESP_RETURN_ON_ERROR(tmc2209_set_microsteps(&motor->tmc, motor->cfg.microsteps),
                        TAG, "set_microsteps failed");

    ESP_RETURN_ON_ERROR(tmc2209_set_stallguard(&motor->tmc,
                                                motor->cfg.sg_threshold,
                                                motor->cfg.sg_tcoolthrs),
                        TAG, "set_stallguard failed");

    // StallGuard on DIAG works ONLY in StealthChop mode (datasheet §5.2).
    // Do not enable SpreadCycle here — use the console command if needed.
    // TPWMTHRS=0: StealthChop active at any speed (TSTEP > 0 always true).
    tmc2209_write_reg(&motor->tmc, TMC2209_REG_TPWMTHRS, 0);

    // TPOWERDOWN: reduce hold current after 2s idle (value * 2^18 / fclk)
    tmc2209_write_reg(&motor->tmc, TMC2209_REG_TPOWERDOWN, 20);

    // Clear GSTAT reset bit so check_and_reinit_tmc won't fire a false positive
    tmc2209_write_reg(&motor->tmc, TMC2209_REG_GSTAT, 0x01);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Check if TMC2209 was power-cycled (GSTAT reset bit) and re-apply config
// ---------------------------------------------------------------------------
static esp_err_t check_and_reinit_tmc(lock_motor_t *motor)
{
    uint32_t gstat;
    if (tmc2209_read_reg(&motor->tmc, TMC2209_REG_GSTAT, &gstat) != ESP_OK)
        return ESP_FAIL;
    if (gstat & 0x01) {  // reset bit set after power-on
        ESP_LOGW(TAG, "TMC2209 power-cycle detected, re-applying config");
        tmc2209_write_reg(&motor->tmc, TMC2209_REG_GSTAT, 0x01); // clear reset bit
        return apply_tmc2209_config(motor);
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
esp_err_t lock_motor_init(lock_motor_t *motor, const lock_motor_config_t *cfg)
{
    if (!motor || !cfg) return ESP_ERR_INVALID_ARG;

    memset(motor, 0, sizeof(*motor));
    motor->cfg = *cfg;

    // Calculate lock_steps if not provided
    if (motor->cfg.lock_steps == 0) {
        motor->cfg.lock_steps = cfg->steps_per_rev
                               * cfg->microsteps
                               * cfg->lock_turns;
    }
    motor->locked_pos   = 0;
    motor->unlocked_pos = motor->cfg.lock_steps;
    motor->state        = LOCK_STATE_UNKNOWN;

    // --- GPIO: STEP, DIR, EN ---
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << cfg->step_gpio)
                      | (1ULL << cfg->dir_gpio)
                      | (1ULL << cfg->en_gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config output failed");

    // Start disabled (EN active LOW: HIGH = disabled)
    gpio_set_level(cfg->en_gpio,   1);
    gpio_set_level(cfg->step_gpio, 0);
    gpio_set_level(cfg->dir_gpio,  0);

    // --- GPIO: DIAG (StallGuard output, rising edge interrupt) ---
    // DIAG is a push-pull output (TMC2209 datasheet): no pull required.
    // LOW during normal operation, HIGH on stall → rising-edge interrupt.
    gpio_config_t diag_io = {
        .pin_bit_mask = (1ULL << cfg->diag_gpio),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&diag_io), TAG, "gpio_config DIAG failed");
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "gpio_install_isr_service");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(cfg->diag_gpio, diag_isr_handler, motor),
                        TAG, "gpio_isr_handler_add DIAG");

    // --- GPTimer for step generation ---
    gptimer_config_t timer_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz → 1 µs resolution
    };
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_cfg, &motor->timer),
                        TAG, "gptimer_new_timer failed");

    gptimer_event_callbacks_t cbs = {
        .on_alarm = step_timer_cb,
    };
    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(motor->timer, &cbs, motor),
                        TAG, "gptimer_register_event_callbacks failed");
    ESP_RETURN_ON_ERROR(gptimer_enable(motor->timer), TAG, "gptimer_enable failed");

    // --- Semaphore for move completion ---
    motor->done_sem = xSemaphoreCreateBinary();
    if (!motor->done_sem) {
        return ESP_ERR_NO_MEM;
    }

    // --- TMC2209 UART init ---
    tmc2209_config_t tmc_cfg = {
        .uart_port  = cfg->tmc_uart_port,
        .tx_gpio    = cfg->tmc_tx_gpio,
        .rx_gpio    = cfg->tmc_rx_gpio,
        .slave_addr = cfg->tmc_slave_addr,
    };
    ESP_RETURN_ON_ERROR(tmc2209_init(&motor->tmc, &tmc_cfg),
                        TAG, "tmc2209_init failed");

    // --- Configure TMC2209 registers ---
    ESP_RETURN_ON_ERROR(apply_tmc2209_config(motor), TAG, "TMC2209 config failed");

    ESP_LOGI(TAG, "Motor initialized: %d steps/rev, %d ustep, %d lock_steps",
             cfg->steps_per_rev, cfg->microsteps, motor->cfg.lock_steps);

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Enable / disable
// ---------------------------------------------------------------------------
esp_err_t lock_motor_enable(lock_motor_t *motor, bool enable)
{
    if (enable) {
        check_and_reinit_tmc(motor);
    }
    // EN pin: active LOW
    gpio_set_level(motor->cfg.en_gpio, enable ? 0 : 1);
    ESP_LOGI(TAG, "Motor %s", enable ? "ENABLED" : "DISABLED");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Internal: arm the timer for N steps
// ---------------------------------------------------------------------------
static esp_err_t start_steps(lock_motor_t *motor, int32_t steps, bool forward)
{
    if (steps <= 0) return ESP_ERR_INVALID_ARG;
    if (motor->running) return ESP_ERR_INVALID_STATE;

    // Set direction GPIO
    motor->dir_fwd = forward;
    gpio_set_level(motor->cfg.dir_gpio, forward ? 1 : 0);
    esp_rom_delay_us(20); // TMC2209 tDH: DIR must be stable before STEP

    // Reset state
    motor->stall_detected  = false;
    motor->steps_remaining = steps;
    motor->running         = true;
    motor->state           = LOCK_STATE_MOVING;

    // Drain any leftover semaphore token
    xSemaphoreTake(motor->done_sem, 0);

    // Configure and start alarm
    gptimer_alarm_config_t alarm = {
        .alarm_count            = motor->cfg.step_period_us,
        .reload_count           = 0,
        .flags.auto_reload_on_alarm = true,
    };
    esp_err_t ret = gptimer_set_alarm_action(motor->timer, &alarm);
    if (ret != ESP_OK) return ret;

    ret = gptimer_set_raw_count(motor->timer, 0);
    if (ret != ESP_OK) return ret;

    return gptimer_start(motor->timer);
}

// ---------------------------------------------------------------------------
// Move (async)
// ---------------------------------------------------------------------------
esp_err_t lock_motor_move(lock_motor_t *motor, int32_t steps, bool forward)
{
    return start_steps(motor, steps, forward);
}

// ---------------------------------------------------------------------------
// Wait for completion
// ---------------------------------------------------------------------------
esp_err_t lock_motor_wait(lock_motor_t *motor, uint32_t timeout_ms)
{
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY
                                         : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(motor->done_sem, ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    gptimer_stop(motor->timer);
    if (motor->state == LOCK_STATE_MOVING) {
        motor->state = LOCK_STATE_UNKNOWN;
    }

    if (motor->stall_detected) {
        ESP_LOGW(TAG, "Move ended by StallGuard at position %"PRId32, motor->position);
    } else {
        ESP_LOGD(TAG, "Move complete at position %"PRId32, motor->position);
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Move and wait
// ---------------------------------------------------------------------------
esp_err_t lock_motor_move_wait(lock_motor_t *motor,
                                int32_t steps, bool forward,
                                uint32_t timeout_ms)
{
    esp_err_t ret = lock_motor_move(motor, steps, forward);
    if (ret != ESP_OK) return ret;
    return lock_motor_wait(motor, timeout_ms);
}

// ---------------------------------------------------------------------------
// Stop
// ---------------------------------------------------------------------------
esp_err_t lock_motor_stop(lock_motor_t *motor)
{
    motor->steps_remaining = 0;
    motor->running         = false;
    motor->state           = LOCK_STATE_UNKNOWN;
    gptimer_stop(motor->timer);
    xSemaphoreGive(motor->done_sem); // unblock any waiting task
    ESP_LOGI(TAG, "Motor stopped at position %"PRId32, motor->position);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Calibrate: measure the real mechanical stroke.
// 1. Move FWD until stall (locked end-stop) → position = 0
// 2. Move REV until stall (unlocked end-stop) → measure actual steps
// Saves measured stroke in cfg.lock_steps for use as safety limit in open/close.
// ---------------------------------------------------------------------------
esp_err_t lock_motor_calibrate(lock_motor_t *motor, bool forward)
{
    const int32_t max_cal_steps = motor->cfg.steps_per_rev
                                * motor->cfg.microsteps
                                * 10; // 10 full turns safety limit

    // Step 1: move to locked end-stop (FWD)
    ESP_LOGI(TAG, "Calibrate step 1: moving FWD to locked end-stop");
    lock_motor_enable(motor, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    lock_motor_move_wait(motor, max_cal_steps, true, 30000);

    if (!motor->stall_detected) {
        ESP_LOGE(TAG, "Calibration failed: no stall on FWD");
        lock_motor_enable(motor, false);
        motor->state = LOCK_STATE_ERROR;
        return ESP_FAIL;
    }
    motor->position   = 0;
    motor->locked_pos = 0;
    ESP_LOGI(TAG, "Locked end-stop found, position zeroed");

    // Brief pause before reversing
    vTaskDelay(pdMS_TO_TICKS(200));

    // Step 2: move to unlocked end-stop (REV)
    ESP_LOGI(TAG, "Calibrate step 2: moving REV to unlocked end-stop");
    motor->stall_detected = false;

    lock_motor_move_wait(motor, max_cal_steps, false, 30000);

    if (!motor->stall_detected) {
        ESP_LOGE(TAG, "Calibration failed: no stall on REV");
        lock_motor_enable(motor, false);
        motor->state = LOCK_STATE_ERROR;
        return ESP_FAIL;
    }

    // Measure actual stroke
    int32_t stroke = -motor->position; // position is negative (moved REV)
    if (stroke <= 0) {
        ESP_LOGE(TAG, "Calibration failed: invalid stroke %"PRId32, stroke);
        lock_motor_enable(motor, false);
        motor->state = LOCK_STATE_ERROR;
        return ESP_FAIL;
    }

    motor->cfg.lock_steps = stroke;
    motor->unlocked_pos   = motor->position;
    motor->state          = LOCK_STATE_UNLOCKED;
    lock_motor_enable(motor, false);

    ESP_LOGI(TAG, "Calibration done: stroke=%"PRId32" microsteps (%.2f turns)",
             stroke, (float)stroke / (motor->cfg.steps_per_rev * motor->cfg.microsteps));
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Open (unlock) — always runs to mechanical end-stop via StallGuard.
// Works regardless of current position; motor is disabled after completion.
// ---------------------------------------------------------------------------
esp_err_t lock_motor_open(lock_motor_t *motor)
{
    // Safety limit: calibrated stroke + 20%, or 5 full turns if not calibrated
    int32_t max_steps = (motor->cfg.lock_steps > 0)
                      ? (motor->cfg.lock_steps * 6 / 5)
                      : (motor->cfg.steps_per_rev * motor->cfg.microsteps * 5);

    ESP_LOGI(TAG, "Opening: REV up to %"PRId32" steps (StallGuard stop)", max_steps);
    lock_motor_enable(motor, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    lock_motor_move_wait(motor, max_steps, false, 30000);
    lock_motor_enable(motor, false);

    if (motor->stall_detected) {
        motor->state = LOCK_STATE_UNLOCKED;
        ESP_LOGI(TAG, "Open: end-stop reached");
        return ESP_OK;
    }

    motor->state = LOCK_STATE_ERROR;
    ESP_LOGE(TAG, "Open: no stall detected — check StallGuard config");
    return ESP_FAIL;
}

// ---------------------------------------------------------------------------
// Close (lock) — always runs to mechanical end-stop via StallGuard.
// Works regardless of current position; motor is disabled after completion.
// ---------------------------------------------------------------------------
esp_err_t lock_motor_close(lock_motor_t *motor)
{
    // Safety limit: calibrated stroke + 20%, or 5 full turns if not calibrated
    int32_t max_steps = (motor->cfg.lock_steps > 0)
                      ? (motor->cfg.lock_steps * 6 / 5)
                      : (motor->cfg.steps_per_rev * motor->cfg.microsteps * 5);

    ESP_LOGI(TAG, "Closing: FWD up to %"PRId32" steps (StallGuard stop)", max_steps);
    lock_motor_enable(motor, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    lock_motor_move_wait(motor, max_steps, true, 30000);
    lock_motor_enable(motor, false);

    if (motor->stall_detected) {
        motor->state = LOCK_STATE_LOCKED;
        ESP_LOGI(TAG, "Close: end-stop reached");
        return ESP_OK;
    }

    motor->state = LOCK_STATE_ERROR;
    ESP_LOGE(TAG, "Close: no stall detected — check StallGuard config");
    return ESP_FAIL;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------
lock_state_t lock_motor_get_state(lock_motor_t *motor)
{
    return motor->state;
}

int32_t lock_motor_get_position(lock_motor_t *motor)
{
    return motor->position;
}

bool lock_motor_is_stalled(lock_motor_t *motor)
{
    return motor->stall_detected;
}

void lock_motor_set_speed(lock_motor_t *motor, uint32_t step_period_us)
{
    motor->cfg.step_period_us = step_period_us;
}

const char *lock_motor_state_str(lock_state_t state)
{
    switch (state) {
        case LOCK_STATE_UNKNOWN:  return "UNKNOWN";
        case LOCK_STATE_LOCKED:   return "LOCKED";
        case LOCK_STATE_UNLOCKED: return "UNLOCKED";
        case LOCK_STATE_MOVING:   return "MOVING";
        case LOCK_STATE_ERROR:    return "ERROR";
        default:                  return "?";
    }
}
