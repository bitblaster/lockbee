/*
 * LockBee — TTP223 touch button handler
 *
 * Two TTP223 modules connected to GPIO 5 (open) and GPIO 6 (close).
 * Output: active HIGH on touch (push-pull, no external pull needed).
 *
 * Architecture:
 *  - GPIO rising-edge ISR → sends button index to a FreeRTOS queue
 *  - touch_task → debounce (500 ms per button) → zigbee_motor_cmd_send()
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

#define DUAL_PRESS_WINDOW_MS  100   /* wait after ISR to detect simultaneous press */
#define DEBOUNCE_MS           500   /* ignore same button for this long after an action */

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

static void touch_task(void *pvParameters)
{
    uint8_t    btn;
    TickType_t last_action[2] = {0, 0};

    while (1) {
        if (xQueueReceive(s_touch_queue, &btn, portMAX_DELAY) != pdTRUE) continue;
        if (btn > 1) continue;

        /* Ignore entirely if motor is already moving */
        if (lock_motor_get_state(s_motor) == LOCK_STATE_MOVING) {
            ESP_LOGI(TAG, "Touch ignored: motor busy");
            continue;
        }

        /* Short wait to catch simultaneous press of the other button */
        vTaskDelay(pdMS_TO_TICKS(DUAL_PRESS_WINDOW_MS));

        int open_lvl  = gpio_get_level(TOUCH_OPEN_GPIO);
        int close_lvl = gpio_get_level(TOUCH_CLOSE_GPIO);

        if (open_lvl && close_lvl) {
            /* Both buttons held: wait 5 s keeping both pressed to confirm reset */
            ESP_LOGI(TAG, "Both buttons held: keep for 5s for Zigbee reset...");
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
                xQueueReset(s_touch_queue);
                zigbee_factory_reset();
            } else {
                ESP_LOGI(TAG, "Dual-press released early: reset cancelled");
            }
            /* Wait for release + debounce regardless */
            vTaskDelay(pdMS_TO_TICKS(2000));
            xQueueReset(s_touch_queue);
            last_action[0] = last_action[1] = xTaskGetTickCount();
            continue;
        }

        /* Per-button debounce */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_action[btn]) < pdMS_TO_TICKS(DEBOUNCE_MS)) continue;
        last_action[btn] = now;

        bool open = (btn == BTN_OPEN);
        ESP_LOGI(TAG, "Touch: %s", open ? "OPEN" : "CLOSE");
        zigbee_motor_cmd_send(open);
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
