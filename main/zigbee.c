/*
 * LockBee — Zigbee integration (ESP32-H2, Door Lock cluster 0x0101)
 *
 * Architecture:
 *  - zb_task()     : Zigbee stack main loop (blocking). Runs forever.
 *  - motor_task()  : Receives motor commands from a FreeRTOS queue, executes
 *                    them (blocking), then schedules a ZCL attribute update
 *                    back into the Zigbee task context via scheduler alarm.
 *
 * Thread-safety notes:
 *  - esp_zb_zcl_set_attribute_val() and esp_zb_zcl_report_attr_cmd_req()
 *    must be called only from the Zigbee task context — always via
 *    esp_zb_scheduler_user_alarm(…, delay=0).
 *  - esp_zb_scheduler_user_alarm() itself is called from motor_task and the
 *    console REPL task; this is intentional and supported by the SDK.
 */

#include "zigbee.h"

#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "platform/esp_zigbee_platform.h"

static const char *TAG = "zigbee";

/* ─── Constants ────────────────────────────────────────────────────────── */

#define DOOR_LOCK_ENDPOINT      1
#define MOTOR_CMD_QUEUE_LEN     3
#define ZB_TASK_STACK_SIZE      4096
#define MOTOR_TASK_STACK_SIZE   4096
#define ZB_TASK_PRIO            5
#define MOTOR_TASK_PRIO         4

/* ZCL Door Lock — LockState attribute values (Table 7-4 ZCL spec) */
#define ZCL_LOCK_STATE_NOT_FULLY_LOCKED  0x00
#define ZCL_LOCK_STATE_LOCKED            0x01
#define ZCL_LOCK_STATE_UNLOCKED          0x02

/* ─── Motor command queue ───────────────────────────────────────────────── */

typedef enum {
    MOTOR_CMD_LOCK,
    MOTOR_CMD_UNLOCK,
    MOTOR_CMD_UNLOCK_WITH_TIMEOUT,
} motor_cmd_type_t;

typedef struct {
    motor_cmd_type_t type;
    uint32_t         timeout_sec;  /* used only for MOTOR_CMD_UNLOCK_WITH_TIMEOUT */
} motor_cmd_t;

/* ─── Module state ──────────────────────────────────────────────────────── */

static QueueHandle_t   s_motor_cmd_queue;
static lock_motor_t   *s_motor;

/*
 * Relock generation counter.
 *
 * Avoids calling esp_zb_scheduler_user_alarm_cancel() from a non-ZB task:
 * motor_task increments s_relock_gen when starting a new command, and
 * relock_cb compares its captured generation before taking action.
 */
static volatile uint32_t  s_relock_gen;
static uint8_t             s_relock_alarm_handle;  /* esp_zb_user_cb_handle_t = uint8 */

/* ─── ZCL attribute update (must run in Zigbee task context) ────────────── */

static void update_lock_state_cb(void *param)
{
    lock_state_t state = (lock_state_t)(uintptr_t)param;
    uint8_t zb_state;

    switch (state) {
    case LOCK_STATE_LOCKED:   zb_state = ZCL_LOCK_STATE_LOCKED;   break;
    case LOCK_STATE_UNLOCKED: zb_state = ZCL_LOCK_STATE_UNLOCKED; break;
    default:                  zb_state = ZCL_LOCK_STATE_NOT_FULLY_LOCKED; break;
    }

    esp_zb_zcl_set_attribute_val(
        DOOR_LOCK_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_DOOR_LOCK_LOCK_STATE_ID,
        &zb_state, false);

    /* Push report to coordinator (short addr 0x0000, endpoint 1) */
    esp_zb_zcl_report_attr_cmd_t report = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint          = 1,
            .src_endpoint          = DOOR_LOCK_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID    = ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
        .attributeID  = ESP_ZB_ZCL_ATTR_DOOR_LOCK_LOCK_STATE_ID,
        .direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
    };
    esp_zb_zcl_report_attr_cmd_req(&report);

    ESP_LOGI(TAG, "ZCL LockState → %d (%s)", zb_state,
             (zb_state == ZCL_LOCK_STATE_LOCKED) ? "LOCKED" :
             (zb_state == ZCL_LOCK_STATE_UNLOCKED) ? "UNLOCKED" : "NOT_FULLY_LOCKED");
}

void zigbee_schedule_lock_state_update(lock_state_t state)
{
    esp_zb_scheduler_user_alarm(update_lock_state_cb, (void *)(uintptr_t)state, 0);
}

/* ─── Factory reset (must run in Zigbee task context) ───────────────────── */

static void factory_reset_cb(void *param)
{
    ESP_LOGW(TAG, "Factory reset: erasing Zigbee network credentials");
    esp_zb_factory_reset();
}

void zigbee_factory_reset(void)
{
    esp_zb_scheduler_user_alarm(factory_reset_cb, NULL, 0);
}

/* ─── Relock timer callback (runs in Zigbee task context) ───────────────── */

static void relock_cb(void *param)
{
    uint32_t gen = (uint32_t)(uintptr_t)param;
    if (gen != s_relock_gen) {
        ESP_LOGI(TAG, "Relock timer expired but superseded (gen %"PRIu32" != %"PRIu32")",
                 gen, s_relock_gen);
        return;
    }
    s_relock_alarm_handle = 0;
    ESP_LOGI(TAG, "Relock timer fired — locking");
    motor_cmd_t cmd = { .type = MOTOR_CMD_LOCK, .timeout_sec = 0 };
    xQueueSend(s_motor_cmd_queue, &cmd, 0);
}

/* ─── ZCL action handler (runs in Zigbee task context) ─────────────────── */

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                    const void *message)
{
    motor_cmd_t cmd = { 0 };
    bool send = false;

    switch (callback_id) {
    case ESP_ZB_CORE_DOOR_LOCK_LOCK_DOOR_CB_ID: {
        const esp_zb_zcl_door_lock_lock_door_message_t *m = message;
        if (m->cmd_id == ESP_ZB_ZCL_CMD_DOOR_LOCK_LOCK_DOOR) {
            cmd.type = MOTOR_CMD_LOCK;
            send = true;
            ESP_LOGI(TAG, "ZCL: Lock command");
        } else if (m->cmd_id == ESP_ZB_ZCL_CMD_DOOR_LOCK_UNLOCK_DOOR) {
            cmd.type = MOTOR_CMD_UNLOCK;
            send = true;
            ESP_LOGI(TAG, "ZCL: Unlock command");
        }
        break;
    }
    case ESP_ZB_CORE_CMD_PRIVILEGE_COMMAND_REQ_CB_ID: {
        const esp_zb_zcl_privilege_command_message_t *m = message;
        if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK && m->size >= 2) {
            /* UnlockWithTimeout payload: 2-byte UINT16 LE = timeout in seconds */
            const uint8_t *d = (const uint8_t *)m->data;
            uint16_t timeout_sec = (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
            cmd.type        = MOTOR_CMD_UNLOCK_WITH_TIMEOUT;
            cmd.timeout_sec = timeout_sec;
            send = true;
            ESP_LOGI(TAG, "ZCL: UnlockWithTimeout %u s", timeout_sec);
        }
        break;
    }
    default:
        break;
    }

    if (send) {
        xQueueSend(s_motor_cmd_queue, &cmd, portMAX_DELAY);
    }
    return ESP_OK;
}

/* ─── Motor task (blocking motor calls, reports state via scheduler) ────── */

static void motor_task(void *pvParameters)
{
    motor_cmd_t cmd;
    while (1) {
        if (xQueueReceive(s_motor_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* Increment generation — any pending relock_cb will see the mismatch
         * and abort. This is the cross-task "cancel" mechanism. */
        uint32_t gen = ++s_relock_gen;

        switch (cmd.type) {
        case MOTOR_CMD_LOCK:
            lock_motor_close(s_motor);
            break;

        case MOTOR_CMD_UNLOCK:
            lock_motor_open(s_motor);
            break;

        case MOTOR_CMD_UNLOCK_WITH_TIMEOUT:
            lock_motor_open(s_motor);
            /* Schedule re-lock. Pass generation so stale callbacks no-op. */
            s_relock_alarm_handle = esp_zb_scheduler_user_alarm(
                relock_cb, (void *)(uintptr_t)gen,
                (uint32_t)cmd.timeout_sec * 1000U);
            ESP_LOGI(TAG, "Relock scheduled in %"PRIu32" s", cmd.timeout_sec);
            break;
        }

        /* Schedule ZCL attribute update in Zigbee task context */
        lock_state_t new_state = lock_motor_get_state(s_motor);
        esp_zb_scheduler_user_alarm(update_lock_state_cb,
                                    (void *)(uintptr_t)new_state, 0);
    }
}

/* ─── Zigbee stack task ─────────────────────────────────────────────────── */

static void zb_task(void *pvParameters)
{
    /* Platform: native 802.15.4 radio, no host connection */
    esp_zb_platform_config_t platform_cfg = {
        .radio_config = { .radio_mode            = ZB_RADIO_MODE_NATIVE },
        .host_config  = { .host_connection_mode  = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_cfg));

    /* Initialize stack as Router (can route traffic and join without coordinator) */
    esp_zb_cfg_t nwk_cfg = {
        .esp_zb_role       = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg  = { .max_children = 10 },
    };
    esp_zb_init(&nwk_cfg);

    /* Create HA Door Lock endpoint */
    esp_zb_door_lock_cfg_t lock_cfg = ESP_ZB_DEFAULT_DOOR_LOCK_CONFIG();
    esp_zb_ep_list_t *ep_list = esp_zb_door_lock_ep_create(DOOR_LOCK_ENDPOINT, &lock_cfg);

    /* Add manufacturer / model info to the Basic cluster.
     * ZCL CharacterString: first byte = length, followed by UTF-8 characters. */
    esp_zb_cluster_list_t *cluster_list =
        esp_zb_ep_list_get_ep(ep_list, DOOR_LOCK_ENDPOINT);
    if (cluster_list) {
        esp_zb_attribute_list_t *basic_cluster = esp_zb_cluster_list_get_cluster(
            cluster_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        if (basic_cluster) {
            static char mfr_name[33], model_id[33];
            snprintf(mfr_name + 1, 32, "%s", CONFIG_LOCKBEE_ZB_MANUFACTURER_NAME);
            mfr_name[0] = (char)strlen(mfr_name + 1);
            snprintf(model_id + 1, 32, "%s", CONFIG_LOCKBEE_ZB_MODEL_IDENTIFIER);
            model_id[0] = (char)strlen(model_id + 1);
            esp_zb_basic_cluster_add_attr(basic_cluster,
                ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, mfr_name);
            esp_zb_basic_cluster_add_attr(basic_cluster,
                ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model_id);
        }
    }

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);

    /* Register UnlockWithTimeout (cmd 0x03) as privilege command so the stack
     * delivers the raw payload instead of silently dropping it. */
    esp_zb_zcl_add_privilege_command(DOOR_LOCK_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
        ESP_ZB_ZCL_CMD_DOOR_LOCK_UNLOCK_WITH_TIMEOUT);

    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

/* ─── Helpers for scheduler_alarm callbacks (must match void(*)(uint8_t)) ── */

static void bdb_init_cb(uint8_t mode)
{
    esp_zb_bdb_start_top_level_commissioning(mode);
}

static void bdb_steering_cb(uint8_t mode)
{
    esp_zb_bdb_start_top_level_commissioning(mode);
}

/* ─── Commissioning signal handler (called from Zigbee task) ────────────── */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p   = signal_struct->p_app_signal;
    esp_err_t err      = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig = *p_sg_p;

    switch (sig) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err == ESP_OK) {
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Factory-new device — starting network steering");
                esp_zb_bdb_start_top_level_commissioning(
                    ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted — syncing lock state to ZCL");
                if (s_motor) {
                    zigbee_schedule_lock_state_update(
                        lock_motor_get_state(s_motor));
                }
            }
        } else {
            ESP_LOGW(TAG, "%s failed (%s) — retrying init in 1 s",
                     esp_zb_zdo_signal_to_string(sig), esp_err_to_name(err));
            esp_zb_scheduler_alarm(bdb_init_cb,
                ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            ESP_LOGI(TAG,
                "Network joined: PAN 0x%04hx  channel %d  short addr 0x%04hx",
                esp_zb_get_pan_id(),
                esp_zb_get_current_channel(),
                esp_zb_get_short_address());
        } else {
            ESP_LOGW(TAG, "Network steering failed (%s) — retrying in 1 s",
                     esp_err_to_name(err));
            esp_zb_scheduler_alarm(bdb_steering_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x) status %s",
                 esp_zb_zdo_signal_to_string(sig), sig, esp_err_to_name(err));
        break;
    }
}

/* ─── Public init ───────────────────────────────────────────────────────── */

esp_err_t zigbee_init(lock_motor_t *motor)
{
    s_motor = motor;
    s_relock_gen          = 0;
    s_relock_alarm_handle = 0;

    s_motor_cmd_queue = xQueueCreate(MOTOR_CMD_QUEUE_LEN, sizeof(motor_cmd_t));
    if (!s_motor_cmd_queue) {
        ESP_LOGE(TAG, "Failed to create motor command queue");
        return ESP_ERR_NO_MEM;
    }

    xTaskCreate(zb_task,     "Zigbee",  ZB_TASK_STACK_SIZE,    NULL, ZB_TASK_PRIO,    NULL);
    xTaskCreate(motor_task,  "ZbMotor", MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIO, NULL);

    ESP_LOGI(TAG, "Zigbee subsystem started (endpoint %d)", DOOR_LOCK_ENDPOINT);
    return ESP_OK;
}
