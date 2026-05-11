/*
 * LockBee — TTP223 touch button handler
 *
 * Two TTP223 modules connected to GPIO 5 (open) and GPIO 6 (close).
 * Output: active HIGH on touch (push-pull, no external pull needed).
 *
 * Architecture:
 *  - GPIO rising-edge ISR → sends button index to a FreeRTOS queue
 *  - touch_task → hold-confirm (500 ms) → zigbee_motor_cmd_send()
 *
 * The motor command is routed through the Zigbee motor queue so that
 * touch, Zigbee, and console commands are all serialised.
 */

#include "touch.h"
#include "zigbee.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "touch";

#define TOUCH_OPEN_GPIO    CONFIG_LOCKBEE_TOUCH_OPEN_GPIO
#define TOUCH_CLOSE_GPIO   CONFIG_LOCKBEE_TOUCH_CLOSE_GPIO

#define TOUCH_QUEUE_LEN       4
#define TOUCH_TASK_STACK      2048
#define TOUCH_TASK_PRIO       3

#define DUAL_PRESS_WINDOW_MS  100   /* wait after first ISR to catch simultaneous press */
#define HOLD_CONFIRM_MS       500   /* button must be held this long to fire the action */

/* Button indices sent through the queue */
#define BTN_OPEN   0
#define BTN_CLOSE  1

static QueueHandle_t  s_touch_queue;
static lock_motor_t  *s_motor;

/* ─── ISR ───────────────────────────────────────────────────────────────── */

static void IRAM_ATTR touch_isr(void *arg)
{
    uint8_t btn = (uint8_t)(uintptr_t)arg;
    xQueueSendFromISR(s_touch_queue, &btn, NULL);
}

/* ─── Task ──────────────────────────────────────────────────────────────── */

/* Wait until a GPIO goes LOW (button released), polling every 50 ms. */
static void wait_for_release(int gpio)
{
    while (gpio_get_level(gpio)) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void touch_task(void *pvParameters)
{
    uint8_t btn;

    while (1) {
        if (xQueueReceive(s_touch_queue, &btn, portMAX_DELAY) != pdTRUE) continue;
        if (btn > 1) continue;

        /* ── Step 1: wait DUAL_PRESS_WINDOW_MS for a second button event ── */
        uint8_t second_btn = 0xFF;
        bool got_second = (xQueueReceive(s_touch_queue, &second_btn,
                                         pdMS_TO_TICKS(DUAL_PRESS_WINDOW_MS)) == pdTRUE);

        if (got_second && second_btn != btn && second_btn <= 1) {
            /* ── Dual press: hold both for 5 s to confirm Zigbee reset ── */
            ESP_LOGI(TAG, "Both buttons pressed: keep for 5s for Zigbee reset...");
            bool confirmed = true;
            for (int i = 0; i < 50; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (!gpio_get_level(TOUCH_OPEN_GPIO) || !gpio_get_level(TOUCH_CLOSE_GPIO)) {
                    confirmed = false;
                    break;
                }
            }
            if (confirmed) {
                ESP_LOGI(TAG, "Confirmed: Zigbee factory reset");
                zigbee_factory_reset();
            } else {
                ESP_LOGI(TAG, "Dual-press released early: reset cancelled");
            }
            /* Wait for both to be released, then drain the queue */
            wait_for_release(TOUCH_OPEN_GPIO);
            wait_for_release(TOUCH_CLOSE_GPIO);
            xQueueReset(s_touch_queue);
            continue;
        }

        /* ── Step 2: single press — confirm by holding HOLD_CONFIRM_MS total ──
         * We already spent DUAL_PRESS_WINDOW_MS waiting; wait the remainder.  */
        int btn_gpio = (btn == BTN_OPEN) ? TOUCH_OPEN_GPIO : TOUCH_CLOSE_GPIO;
        vTaskDelay(pdMS_TO_TICKS(HOLD_CONFIRM_MS - DUAL_PRESS_WINDOW_MS));

        if (!gpio_get_level(btn_gpio)) {
            /* Released before HOLD_CONFIRM_MS: accidental touch, ignore */
            xQueueReset(s_touch_queue);
            continue;
        }

        /* ── Step 3: button confirmed held — fire action ── */
        if (lock_motor_get_state(s_motor) == LOCK_STATE_MOVING) {
            ESP_LOGI(TAG, "Touch ignored: motor busy");
        } else {
            bool open = (btn == BTN_OPEN);
            ESP_LOGI(TAG, "Touch: %s", open ? "OPEN" : "CLOSE");
            zigbee_motor_cmd_send(open);
        }

        /* ── Step 4: wait for release so holding never re-triggers ── */
        wait_for_release(btn_gpio);
        xQueueReset(s_touch_queue);
    }
}

/* ─── Init ──────────────────────────────────────────────────────────────── */

esp_err_t touch_init(lock_motor_t *motor)
{
    s_motor = motor;
    s_touch_queue = xQueueCreate(TOUCH_QUEUE_LEN, sizeof(uint8_t));
    if (!s_touch_queue) {
        return ESP_ERR_NO_MEM;
    }

    /* Both TTP223 outputs are push-pull active HIGH — no pull resistor needed */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << TOUCH_OPEN_GPIO) | (1ULL << TOUCH_CLOSE_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config failed");

    /* gpio_install_isr_service is already called in lock_motor_init */
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(TOUCH_OPEN_GPIO,  touch_isr, (void *)(uintptr_t)BTN_OPEN),
        TAG, "isr_handler_add OPEN failed");
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(TOUCH_CLOSE_GPIO, touch_isr, (void *)(uintptr_t)BTN_CLOSE),
        TAG, "isr_handler_add CLOSE failed");

    xTaskCreate(touch_task, "Touch", TOUCH_TASK_STACK, NULL, TOUCH_TASK_PRIO, NULL);

    ESP_LOGI(TAG, "Touch buttons: OPEN=GPIO%d  CLOSE=GPIO%d",
             TOUCH_OPEN_GPIO, TOUCH_CLOSE_GPIO);
    return ESP_OK;
}
