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

#define TOUCH_QUEUE_LEN    4
#define TOUCH_TASK_STACK   2048
#define TOUCH_TASK_PRIO    3

/* Button indices sent through the queue */
#define BTN_OPEN   0
#define BTN_CLOSE  1

static QueueHandle_t s_touch_queue;

/* ─── ISR ───────────────────────────────────────────────────────────────── */

static void IRAM_ATTR touch_isr(void *arg)
{
    uint8_t btn = (uint8_t)(uintptr_t)arg;
    xQueueSendFromISR(s_touch_queue, &btn, NULL);
}

/* ─── Task ──────────────────────────────────────────────────────────────── */

static void touch_task(void *pvParameters)
{
    uint8_t btn;

    while (1) {
        if (xQueueReceive(s_touch_queue, &btn, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (btn > 1) continue;

        bool open = (btn == BTN_OPEN);
        ESP_LOGI(TAG, "Touch: %s", open ? "OPEN" : "CLOSE");
        zigbee_motor_cmd_send(open);
    }
}

/* ─── Init ──────────────────────────────────────────────────────────────── */

esp_err_t touch_init(void)
{
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
