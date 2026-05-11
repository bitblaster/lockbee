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
 *  - Any esp_zb_scheduler_user_alarm() call from a non-ZB task must be
 *    wrapped with esp_zb_lock_acquire()/esp_zb_lock_release() to serialise
 *    with the ZBOSS kernel and prevent critical-section imbalance crashes.
 */

#include "zigbee.h"
#include "touch.h"

#include <string.h>
#include <stdint.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_ota_ops.h"
#include "esp_zigbee_ota.h"

#define OTA_ELEMENT_HEADER_LEN  6   /* uint16_t tag + uint32_t length */

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

/* OTA Upgrade client */
#define OTA_MANUFACTURER_CODE   0x1234   /* arbitrary; must match .ota file header */
#define OTA_IMAGE_TYPE          0x0000
#define OTA_FILE_VERSION        0x00000001
#define OTA_MAX_DATA_SIZE       64

/* ZCL Door Lock — LockState attribute values (Table 7-4 ZCL spec) */
#define ZCL_LOCK_STATE_NOT_FULLY_LOCKED  0x00
#define ZCL_LOCK_STATE_LOCKED            0x01
#define ZCL_LOCK_STATE_UNLOCKED          0x02

/* ZCL Door Lock — EnableOneTouchLocking attribute (boolean, R/W) */
#define ZCL_ATTR_ENABLE_ONE_TOUCH_LOCKING  0x0029

#define NVS_NAMESPACE       "lockbee"
#define NVS_KEY_TOUCH_EN    "touch_en"

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

static QueueHandle_t         s_motor_cmd_queue;
static lock_motor_t         *s_motor;

/* OTA state (used only during an active upgrade) */
static esp_ota_handle_t      s_ota_handle;
static const esp_partition_t *s_ota_partition;

/*
 * Relock generation counter.
 *
 * Avoids calling esp_zb_scheduler_user_alarm_cancel() from a non-ZB task:
 * motor_task increments s_relock_gen when starting a new command, and
 * relock_cb compares its captured generation before taking action.
 */
static volatile uint32_t  s_relock_gen;
static uint8_t             s_relock_alarm_handle;  /* esp_zb_user_cb_handle_t = uint8 */

/* ─── OTA Upgrade callback ──────────────────────────────────────────────── */

static esp_err_t zb_ota_cb(esp_zb_zcl_ota_upgrade_value_message_t msg)
{
    static uint32_t s_total_size = 0;
    static uint32_t s_offset     = 0;
    static bool     s_hdr_done   = false;
    esp_err_t ret = ESP_OK;

    if (msg.info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_FAIL;
    }

    switch (msg.upgrade_status) {

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
        ESP_LOGI(TAG, "OTA upgrade started (file ver 0x%08"PRIx32")",
                 msg.ota_header.file_version);
        s_ota_partition = esp_ota_get_next_update_partition(NULL);
        if (!s_ota_partition) {
            ESP_LOGE(TAG, "OTA: no update partition available");
            return ESP_FAIL;
        }
        ret = esp_ota_begin(s_ota_partition, 0, &s_ota_handle);
        ESP_RETURN_ON_ERROR(ret, TAG, "esp_ota_begin failed");
        s_total_size = 0;
        s_offset     = 0;
        s_hdr_done   = false;
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
        s_total_size = msg.ota_header.image_size;
        if (msg.payload && msg.payload_size) {
            const uint8_t *data = (const uint8_t *)msg.payload;
            uint16_t       dlen = msg.payload_size;
            /* Strip 6-byte OTA element header (uint16_t tag + uint32_t length)
             * that Zigbee2MQTT prepends to the first block. */
            if (!s_hdr_done) {
                s_hdr_done = true;
                if (dlen > OTA_ELEMENT_HEADER_LEN) {
                    data += OTA_ELEMENT_HEADER_LEN;
                    dlen -= OTA_ELEMENT_HEADER_LEN;
                } else {
                    /* Header-only block — nothing to write */
                    s_offset += dlen;
                    break;
                }
            }
            s_offset += msg.payload_size;
            ESP_LOGI(TAG, "OTA receive: %"PRIu32"/%"PRIu32" bytes", s_offset, s_total_size);
            ret = esp_ota_write(s_ota_handle, data, dlen);
            ESP_RETURN_ON_ERROR(ret, TAG, "esp_ota_write failed");
        }
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
        ESP_LOGI(TAG, "OTA upgrade apply");
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
        ret = (s_offset == s_total_size) ? ESP_OK : ESP_FAIL;
        ESP_LOGI(TAG, "OTA check: offset=%"PRIu32" total=%"PRIu32" → %s",
                 s_offset, s_total_size, esp_err_to_name(ret));
        s_offset = 0; s_total_size = 0; s_hdr_done = false;
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
        ESP_LOGI(TAG, "OTA complete — setting boot partition and rebooting");
        ret = esp_ota_end(s_ota_handle);
        ESP_RETURN_ON_ERROR(ret, TAG, "esp_ota_end failed");
        ret = esp_ota_set_boot_partition(s_ota_partition);
        ESP_RETURN_ON_ERROR(ret, TAG, "esp_ota_set_boot_partition failed");
        esp_restart();
        break;

    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
        ESP_LOGW(TAG, "OTA upgrade aborted");
        esp_ota_abort(s_ota_handle);
        s_ota_handle    = 0;
        s_ota_partition = NULL;
        s_offset = 0; s_total_size = 0; s_hdr_done = false;
        break;

    default:
        ESP_LOGI(TAG, "OTA status: %d", msg.upgrade_status);
        break;
    }
    return ret;
}

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
    /* Must NOT be called from within the ZB task (would deadlock on lock).
     * For ZB-task-internal use call update_lock_state_cb() directly. */
    if (esp_zb_lock_acquire(portMAX_DELAY)) {
        esp_zb_scheduler_user_alarm(update_lock_state_cb,
                                    (void *)(uintptr_t)state, 0);
        esp_zb_lock_release();
    }
}

/* ─── Factory reset (must run in Zigbee task context) ───────────────────── */

static void factory_reset_cb(void *param)
{
    ESP_LOGW(TAG, "Factory reset: erasing Zigbee network credentials");
    esp_zb_factory_reset();
}

void zigbee_factory_reset(void)
{
    if (esp_zb_lock_acquire(portMAX_DELAY)) {
        esp_zb_scheduler_user_alarm(factory_reset_cb, NULL, 0);
        esp_zb_lock_release();
    }
}

/* ─── Motor command send (thread/ISR-safe) ──────────────────────────────── */

void zigbee_motor_cmd_send(bool open)
{
    motor_cmd_t cmd = { .type = open ? MOTOR_CMD_UNLOCK : MOTOR_CMD_LOCK };
    xQueueSend(s_motor_cmd_queue, &cmd, 0);
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
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: {
        const esp_zb_zcl_set_attr_value_message_t *m = message;
        if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK &&
            m->attribute.id == ZCL_ATTR_ENABLE_ONE_TOUCH_LOCKING) {
            uint8_t val = *(uint8_t *)m->attribute.data.value;
            touch_set_enabled(val != 0);
            nvs_handle_t h;
            if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_u8(h, NVS_KEY_TOUCH_EN, val);
                nvs_commit(h);
                nvs_close(h);
            }
        }
        break;
    }
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        return zb_ota_cb(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);

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
            if (esp_zb_lock_acquire(portMAX_DELAY)) {
                s_relock_alarm_handle = esp_zb_scheduler_user_alarm(
                    relock_cb, (void *)(uintptr_t)gen,
                    (uint32_t)cmd.timeout_sec * 1000U);
                esp_zb_lock_release();
            }
            ESP_LOGI(TAG, "Relock scheduled in %"PRIu32" s", cmd.timeout_sec);
            break;
        }

        /* Schedule ZCL attribute update in Zigbee task context.
         * esp_zb_lock_acquire serialises with the ZBOSS kernel so that
         * esp_zb_scheduler_user_alarm() is safe to call from this task. */
        lock_state_t new_state = lock_motor_get_state(s_motor);
        if (esp_zb_lock_acquire(portMAX_DELAY)) {
            esp_zb_scheduler_user_alarm(update_lock_state_cb,
                                        (void *)(uintptr_t)new_state, 0);
            esp_zb_lock_release();
        }
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

    esp_zb_cluster_list_t *cluster_list =
        esp_zb_ep_list_get_ep(ep_list, DOOR_LOCK_ENDPOINT);

    /* Add EnableOneTouchLocking (0x0029) to Door Lock cluster.
     * Boolean R/W attribute — controls whether touch buttons are active. */
    if (cluster_list) {
        esp_zb_attribute_list_t *lock_cluster = esp_zb_cluster_list_get_cluster(
            cluster_list, ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        if (lock_cluster) {
            static uint8_t touch_enabled_attr = 1; /* true = enabled */
            esp_zb_door_lock_cluster_add_attr(lock_cluster,
                ZCL_ATTR_ENABLE_ONE_TOUCH_LOCKING, &touch_enabled_attr);
        }
    }

    /* Add manufacturer / model info to the Basic cluster.
     * ZCL CharacterString: first byte = length, followed by UTF-8 characters. */
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

    /* Add OTA Upgrade client cluster */
    if (cluster_list) {
        esp_zb_ota_cluster_cfg_t ota_cfg = {
            .ota_upgrade_file_version        = OTA_FILE_VERSION,
            .ota_upgrade_downloaded_file_ver = OTA_FILE_VERSION,
            .ota_upgrade_manufacturer        = OTA_MANUFACTURER_CODE,
            .ota_upgrade_image_type          = OTA_IMAGE_TYPE,
        };
        esp_zb_attribute_list_t *ota_attr = esp_zb_ota_cluster_create(&ota_cfg);
        esp_zb_zcl_ota_upgrade_client_variable_t variable_cfg = {
            .timer_query   = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
            .hw_version    = 0x0001,
            .max_data_size = OTA_MAX_DATA_SIZE,
        };
        uint16_t ota_server_addr = 0xffff;
        uint8_t  ota_server_ep   = 0xff;
        esp_zb_ota_cluster_add_attr(ota_attr, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID,      &variable_cfg);
        esp_zb_ota_cluster_add_attr(ota_attr, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID,      &ota_server_addr);
        esp_zb_ota_cluster_add_attr(ota_attr, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID,  &ota_server_ep);
        esp_zb_cluster_list_add_ota_cluster(cluster_list, ota_attr, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
        ESP_LOGI(TAG, "OTA cluster added (manufacturer=0x%04x image=0x%04x ver=0x%08"PRIx32")",
                 OTA_MANUFACTURER_CODE, OTA_IMAGE_TYPE, (uint32_t)OTA_FILE_VERSION);
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
            /* Restore touch-enabled state from NVS */
            {
                nvs_handle_t h;
                if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
                    uint8_t val = 1;
                    nvs_get_u8(h, NVS_KEY_TOUCH_EN, &val);
                    nvs_close(h);
                    touch_set_enabled(val != 0);
                    /* Sync ZCL attribute to match NVS value */
                    esp_zb_zcl_set_attribute_val(
                        DOOR_LOCK_ENDPOINT,
                        ESP_ZB_ZCL_CLUSTER_ID_DOOR_LOCK,
                        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        ZCL_ATTR_ENABLE_ONE_TOUCH_LOCKING,
                        &val, false);
                }
            }
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Factory-new device — starting network steering");
                esp_zb_bdb_start_top_level_commissioning(
                    ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted — syncing lock state to ZCL");
                if (s_motor) {
                    /* Already in ZB task context — call directly, no lock needed */
                    update_lock_state_cb((void *)(uintptr_t)
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

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "Leave request received — clearing credentials and restarting pairing");
        esp_zb_factory_reset();
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
