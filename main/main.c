#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "lock_motor.h"
#include "tmc2209.h"
#include "zigbee.h"
#include "touch.h"

static const char *TAG = "main";

#define NVS_NAMESPACE      "lockbee"
#define NVS_KEY_SPEED      "speed"
#define NVS_KEY_IRUN       "irun"
#define NVS_KEY_IHOLD      "ihold"
#define NVS_KEY_LOCK_STEPS "lock_steps"

static void nvs_save_speed(uint32_t period_us)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, NVS_KEY_SPEED, period_us);
        nvs_commit(h);
        nvs_close(h);
    }
}

static uint32_t nvs_load_speed(uint32_t default_val)
{
    nvs_handle_t h;
    uint32_t val = default_val;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, NVS_KEY_SPEED, &val);
        nvs_close(h);
    }
    return val;
}

static void nvs_save_u8(const char *key, uint8_t val)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, key, val);
        nvs_commit(h);
        nvs_close(h);
    }
}

static uint8_t nvs_load_u8(const char *key, uint8_t default_val)
{
    nvs_handle_t h;
    uint8_t val = default_val;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, key, &val);
        nvs_close(h);
    }
    return val;
}

static void nvs_save_u32(const char *key, uint32_t val)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, key, val);
        nvs_commit(h);
        nvs_close(h);
    }
}

static uint32_t nvs_load_u32(const char *key, uint32_t default_val)
{
    nvs_handle_t h;
    uint32_t val = default_val;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, key, &val);
        nvs_close(h);
    }
    return val;
}

// ---------------------------------------------------------------------------
// Global motor instance
// ---------------------------------------------------------------------------
static lock_motor_t g_motor;

// ---------------------------------------------------------------------------
// Helper: build default config from Kconfig
// ---------------------------------------------------------------------------
static lock_motor_config_t make_default_config(void)
{
    lock_motor_config_t cfg = {
        .step_gpio       = CONFIG_LOCKBEE_STEP_GPIO,
        .dir_gpio        = CONFIG_LOCKBEE_DIR_GPIO,
        .en_gpio         = CONFIG_LOCKBEE_EN_GPIO,
        .diag_gpio       = CONFIG_LOCKBEE_DIAG_GPIO,

        .steps_per_rev   = CONFIG_LOCKBEE_MOTOR_STEPS_PER_REV,
        .microsteps      = CONFIG_LOCKBEE_MICROSTEPS,
        .lock_steps      = CONFIG_LOCKBEE_LOCK_STEPS,
        .lock_turns      = CONFIG_LOCKBEE_LOCK_TURNS,

        .step_period_us  = CONFIG_LOCKBEE_STEP_PERIOD_US,
        .accel_steps     = 0,

        .irun            = CONFIG_LOCKBEE_MOTOR_IRUN,
        .ihold           = CONFIG_LOCKBEE_MOTOR_IHOLD,
        .ihold_delay     = CONFIG_LOCKBEE_MOTOR_IHOLDDELAY,

        .sg_threshold    = CONFIG_LOCKBEE_SG_THRESHOLD,
        .sg_tcoolthrs    = CONFIG_LOCKBEE_SG_TCOOLTHRS,

        .tmc_uart_port   = CONFIG_LOCKBEE_TMC_UART_PORT,
        .tmc_tx_gpio     = CONFIG_LOCKBEE_UART_TX_GPIO,
        .tmc_rx_gpio     = CONFIG_LOCKBEE_UART_RX_GPIO,
        .tmc_slave_addr  = CONFIG_LOCKBEE_TMC_SLAVE_ADDR,
    };
    return cfg;
}

// ============================================================================
// Console commands
// ============================================================================

// ---------------------------------------------------------------------------
// motor open
// ---------------------------------------------------------------------------
static int cmd_open(int argc, char **argv)
{
    ESP_LOGI(TAG, "cmd: open");
    esp_err_t ret = lock_motor_open(&g_motor);
    if (ret != ESP_OK) {
        printf("ERROR: open failed (%s)\n", esp_err_to_name(ret));
        return 1;
    }
    zigbee_schedule_lock_state_update(lock_motor_get_state(&g_motor));
    printf("OK: state=%s pos=%"PRId32"\n",
           lock_motor_state_str(lock_motor_get_state(&g_motor)),
           lock_motor_get_position(&g_motor));
    return 0;
}

// ---------------------------------------------------------------------------
// motor close
// ---------------------------------------------------------------------------
static int cmd_close(int argc, char **argv)
{
    ESP_LOGI(TAG, "cmd: close");
    esp_err_t ret = lock_motor_close(&g_motor);
    if (ret != ESP_OK) {
        printf("ERROR: close failed (%s)\n", esp_err_to_name(ret));
        return 1;
    }
    zigbee_schedule_lock_state_update(lock_motor_get_state(&g_motor));
    printf("OK: state=%s pos=%"PRId32"\n",
           lock_motor_state_str(lock_motor_get_state(&g_motor)),
           lock_motor_get_position(&g_motor));
    return 0;
}

// ---------------------------------------------------------------------------
// motor stop
// ---------------------------------------------------------------------------
static int cmd_stop(int argc, char **argv)
{
    lock_motor_stop(&g_motor);
    printf("OK: stopped at pos=%"PRId32"\n", lock_motor_get_position(&g_motor));
    return 0;
}

// ---------------------------------------------------------------------------
// motor enable / disable
// ---------------------------------------------------------------------------
static struct {
    struct arg_str *action;
    struct arg_end *end;
} enable_args;

static int cmd_enable(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&enable_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, enable_args.end, argv[0]);
        return 1;
    }
    bool en = (strcmp(enable_args.action->sval[0], "on") == 0);
    lock_motor_enable(&g_motor, en);
    printf("OK: motor %s\n", en ? "enabled" : "disabled");
    return 0;
}

// ---------------------------------------------------------------------------
// step <count> [fwd|rev]
// ---------------------------------------------------------------------------
static struct {
    struct arg_int *count;
    struct arg_str *dir;
    struct arg_end *end;
} step_args;

static int cmd_step(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&step_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, step_args.end, argv[0]);
        return 1;
    }

    int32_t n = step_args.count->ival[0];
    bool fwd  = true;
    if (step_args.dir->count > 0) {
        fwd = (strcmp(step_args.dir->sval[0], "rev") != 0);
    }

    printf("Stepping %"PRId32" microsteps %s...\n", n, fwd ? "FWD" : "REV");
    lock_motor_enable(&g_motor, true);

    esp_err_t ret = lock_motor_move_wait(&g_motor, n, fwd, 30000);
    printf("Done: ret=%s stall=%d pos=%"PRId32"\n",
           esp_err_to_name(ret),
           (int)lock_motor_is_stalled(&g_motor),
           lock_motor_get_position(&g_motor));
    return (ret == ESP_OK) ? 0 : 1;
}

// ---------------------------------------------------------------------------
// calibrate [fwd|rev]
// ---------------------------------------------------------------------------
static int cmd_calibrate(int argc, char **argv)
{
    printf("Calibrating: FWD to locked end-stop, then REV to unlocked end-stop...\n");
    esp_err_t ret = lock_motor_calibrate(&g_motor, true);
    if (ret != ESP_OK) {
        printf("ERROR: calibration failed (%s)\n", esp_err_to_name(ret));
        return 1;
    }
    nvs_save_u32(NVS_KEY_LOCK_STEPS, (uint32_t)g_motor.cfg.lock_steps);
    printf("OK: stroke=%d microsteps (%.2f turns) — saved to NVS\n",
           g_motor.cfg.lock_steps,
           (float)g_motor.cfg.lock_steps / (g_motor.cfg.steps_per_rev * g_motor.cfg.microsteps));
    return 0;
}

// ---------------------------------------------------------------------------
// speed <period_us>
// ---------------------------------------------------------------------------
static struct {
    struct arg_int *period;
    struct arg_end *end;
} speed_args;

static int cmd_speed(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&speed_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, speed_args.end, argv[0]);
        return 1;
    }
    uint32_t period = (uint32_t)speed_args.period->ival[0];
    lock_motor_set_speed(&g_motor, period);
    nvs_save_speed(period);
    printf("OK: step_period=%"PRIu32" µs (saved)\n", period);
    return 0;
}

// ---------------------------------------------------------------------------
// status  — show motor state + TMC2209 key registers
// ---------------------------------------------------------------------------
static int cmd_status(int argc, char **argv)
{
    printf("=== LockBee Status ===\n");
    printf("  State    : %s\n", lock_motor_state_str(lock_motor_get_state(&g_motor)));
    printf("  Position : %"PRId32" microsteps\n", lock_motor_get_position(&g_motor));
    printf("  Running  : %s\n", g_motor.running ? "yes" : "no");
    printf("  Stall    : %s\n", lock_motor_is_stalled(&g_motor) ? "yes" : "no");
    printf("  Speed    : %"PRIu32" µs/step\n", g_motor.cfg.step_period_us);
    printf("  Lock pos : locked=%"PRId32" unlocked=%"PRId32"\n",
           g_motor.locked_pos, g_motor.unlocked_pos);

    uint32_t drv_status;
    if (tmc2209_get_drv_status(&g_motor.tmc, &drv_status) == ESP_OK) {
        printf("  DRV_STATUS: 0x%08"PRIx32"\n", drv_status);
        printf("    OT=%d OTPW=%d S2GA=%d S2GB=%d STST=%d CS_ACTUAL=%"PRIu32"\n",
               !!(drv_status & TMC2209_DRV_STATUS_OT),
               !!(drv_status & TMC2209_DRV_STATUS_OTPW),
               !!(drv_status & TMC2209_DRV_STATUS_S2GA),
               !!(drv_status & TMC2209_DRV_STATUS_S2GB),
               !!(drv_status & TMC2209_DRV_STATUS_STST),
               (drv_status & TMC2209_DRV_STATUS_CS_ACT_MASK)
                   >> TMC2209_DRV_STATUS_CS_ACT_SHIFT);
    }

    uint16_t sg;
    if (tmc2209_get_sg_result(&g_motor.tmc, &sg) == ESP_OK) {
        printf("  SG_RESULT : %u (0=stall, 510=no load)\n", sg);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// tmc_read <reg_hex>
// ---------------------------------------------------------------------------
static struct {
    struct arg_str *reg;
    struct arg_end *end;
} tmc_read_args;

static int cmd_tmc_read(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tmc_read_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, tmc_read_args.end, argv[0]);
        return 1;
    }
    uint8_t reg = (uint8_t)strtol(tmc_read_args.reg->sval[0], NULL, 0);
    uint32_t val;
    esp_err_t ret = tmc2209_read_reg(&g_motor.tmc, reg, &val);
    if (ret != ESP_OK) {
        printf("ERROR: read reg 0x%02x: %s\n", reg, esp_err_to_name(ret));
        return 1;
    }
    printf("REG 0x%02x = 0x%08"PRIx32" (%"PRIu32")\n", reg, val, val);
    return 0;
}

// ---------------------------------------------------------------------------
// tmc_write <reg_hex> <value_hex>
// ---------------------------------------------------------------------------
static struct {
    struct arg_str *reg;
    struct arg_str *val;
    struct arg_end *end;
} tmc_write_args;

static int cmd_tmc_write(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&tmc_write_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, tmc_write_args.end, argv[0]);
        return 1;
    }
    uint8_t  reg = (uint8_t) strtol(tmc_write_args.reg->sval[0], NULL, 0);
    uint32_t val = (uint32_t)strtol(tmc_write_args.val->sval[0], NULL, 0);

    esp_err_t ret = tmc2209_write_reg(&g_motor.tmc, reg, val);
    if (ret != ESP_OK) {
        printf("ERROR: write reg 0x%02x: %s\n", reg, esp_err_to_name(ret));
        return 1;
    }
    printf("OK: wrote 0x%08"PRIx32" to reg 0x%02x\n", val, reg);

    // Persist IRUN/IHOLD when writing IHOLD_IRUN register (0x10)
    if (reg == TMC2209_REG_IHOLD_IRUN) {
        uint8_t new_irun  = (val >>  8) & 0x1F;
        uint8_t new_ihold = (val >>  0) & 0x1F;
        g_motor.cfg.irun  = new_irun;
        g_motor.cfg.ihold = new_ihold;
        nvs_save_u8(NVS_KEY_IRUN,  new_irun);
        nvs_save_u8(NVS_KEY_IHOLD, new_ihold);
        ESP_LOGI(TAG, "IRUN=%u IHOLD=%u saved to NVS", new_irun, new_ihold);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// tmc_dump  — dump all key registers
// ---------------------------------------------------------------------------
static int cmd_tmc_dump(int argc, char **argv)
{
    tmc2209_dump_regs(&g_motor.tmc);
    return 0;
}

// ---------------------------------------------------------------------------
// sg_monitor <steps> [fwd|rev]
// Starts the move asynchronously and reads SG_RESULT every 100ms while running.
// CSV output: sample, SG_RESULT, position
// ---------------------------------------------------------------------------
static struct {
    struct arg_int *count;
    struct arg_str *dir;
    struct arg_end *end;
} sg_mon_args;

static int cmd_sg_monitor(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&sg_mon_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, sg_mon_args.end, argv[0]);
        return 1;
    }

    int32_t n = sg_mon_args.count->ival[0];
    bool fwd = true;
    if (sg_mon_args.dir->count > 0) {
        fwd = (strcmp(sg_mon_args.dir->sval[0], "rev") != 0);
    }

    printf("sg_monitor: %"PRId32" steps %s — SG_RESULT every 100ms\n",
           n, fwd ? "FWD" : "REV");
    printf("sample,SG_RESULT,pos\n");

    lock_motor_enable(&g_motor, true);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t ret = lock_motor_move(&g_motor, n, fwd);
    if (ret != ESP_OK) {
        printf("ERROR: move failed (%s)\n", esp_err_to_name(ret));
        lock_motor_enable(&g_motor, false);
        return 1;
    }

    uint32_t sample = 0;
    while (g_motor.running) {
        uint16_t sg = 9999;
        tmc2209_get_sg_result(&g_motor.tmc, &sg);
        printf("%"PRIu32",%u,%"PRId32"\n", sample++, sg, g_motor.position);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    lock_motor_wait(&g_motor, 1000);
    lock_motor_enable(&g_motor, false);

    printf("DONE: stall=%d pos=%"PRId32"\n",
           (int)lock_motor_is_stalled(&g_motor),
           lock_motor_get_position(&g_motor));
    return 0;
}

// ---------------------------------------------------------------------------
// sg_config <threshold> [tcoolthrs]
// ---------------------------------------------------------------------------
static struct {
    struct arg_int *threshold;
    struct arg_int *tcoolthrs;
    struct arg_end *end;
} sg_args;

static int cmd_sg_config(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&sg_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, sg_args.end, argv[0]);
        return 1;
    }
    uint8_t  thr  = (uint8_t)sg_args.threshold->ival[0];
    uint32_t tcool = (sg_args.tcoolthrs->count > 0)
                   ? (uint32_t)sg_args.tcoolthrs->ival[0]
                   : g_motor.cfg.sg_tcoolthrs;

    esp_err_t ret = tmc2209_set_stallguard(&g_motor.tmc, thr, tcool);
    if (ret != ESP_OK) {
        printf("ERROR: %s\n", esp_err_to_name(ret));
        return 1;
    }
    g_motor.cfg.sg_threshold = thr;
    g_motor.cfg.sg_tcoolthrs = tcool;
    printf("OK: SGTHRS=%u TCOOLTHRS=%"PRIu32"\n", thr, tcool);
    return 0;
}

// ---------------------------------------------------------------------------
// zb_reset  — factory-reset Zigbee network credentials and re-pair
// ---------------------------------------------------------------------------
static int cmd_zb_reset(int argc, char **argv)
{
    printf("Scheduling Zigbee factory reset...\n");
    zigbee_factory_reset();
    printf("OK: device will leave network and restart pairing\n");
    return 0;
}

// ---------------------------------------------------------------------------
// Register all commands
// ---------------------------------------------------------------------------
static void register_commands(void)
{
    // open
    esp_console_cmd_t open_cmd = {
        .command = "open",
        .help    = "Unlock the lock (move to unlocked position)",
        .func    = cmd_open,
    };
    esp_console_cmd_register(&open_cmd);

    // close
    esp_console_cmd_t close_cmd = {
        .command = "close",
        .help    = "Lock the lock (move to locked position)",
        .func    = cmd_close,
    };
    esp_console_cmd_register(&close_cmd);

    // stop
    esp_console_cmd_t stop_cmd = {
        .command = "stop",
        .help    = "Stop motor immediately",
        .func    = cmd_stop,
    };
    esp_console_cmd_register(&stop_cmd);

    // enable <on|off>
    enable_args.action = arg_str1(NULL, NULL, "<on|off>", "Enable or disable motor driver");
    enable_args.end    = arg_end(1);
    esp_console_cmd_t enable_cmd = {
        .command = "enable",
        .help    = "Enable or disable motor driver (on/off)",
        .argtable = &enable_args,
        .func    = cmd_enable,
    };
    esp_console_cmd_register(&enable_cmd);

    // step <count> [fwd|rev]
    step_args.count = arg_int1(NULL, NULL, "<steps>", "Number of microsteps");
    step_args.dir   = arg_str0(NULL, NULL, "[fwd|rev]", "Direction (default: fwd)");
    step_args.end   = arg_end(2);
    esp_console_cmd_t step_cmd = {
        .command  = "step",
        .help     = "Move N microsteps in given direction",
        .argtable = &step_args,
        .func     = cmd_step,
    };
    esp_console_cmd_register(&step_cmd);

    // calibrate
    esp_console_cmd_t cal_cmd = {
        .command  = "calibrate",
        .help     = "Measure stroke: FWD to locked end-stop, REV to unlocked end-stop",
        .func     = cmd_calibrate,
    };
    esp_console_cmd_register(&cal_cmd);

    // speed <period_us>
    speed_args.period = arg_int1(NULL, NULL, "<us>", "Step period in microseconds");
    speed_args.end    = arg_end(1);
    esp_console_cmd_t speed_cmd = {
        .command  = "speed",
        .help     = "Set step period in µs (e.g. 2000 = slow, 500 = fast)",
        .argtable = &speed_args,
        .func     = cmd_speed,
    };
    esp_console_cmd_register(&speed_cmd);

    // status
    esp_console_cmd_t status_cmd = {
        .command = "status",
        .help    = "Show motor state and TMC2209 driver status",
        .func    = cmd_status,
    };
    esp_console_cmd_register(&status_cmd);

    // tmc_read <reg>
    tmc_read_args.reg = arg_str1(NULL, NULL, "<reg>", "Register address (hex, e.g. 0x6F)");
    tmc_read_args.end = arg_end(1);
    esp_console_cmd_t tmc_read_cmd = {
        .command  = "tmc_read",
        .help     = "Read a TMC2209 register (e.g. tmc_read 0x6F)",
        .argtable = &tmc_read_args,
        .func     = cmd_tmc_read,
    };
    esp_console_cmd_register(&tmc_read_cmd);

    // tmc_write <reg> <val>
    tmc_write_args.reg = arg_str1(NULL, NULL, "<reg>", "Register address (hex)");
    tmc_write_args.val = arg_str1(NULL, NULL, "<val>", "Value (hex)");
    tmc_write_args.end = arg_end(2);
    esp_console_cmd_t tmc_write_cmd = {
        .command  = "tmc_write",
        .help     = "Write a TMC2209 register (e.g. tmc_write 0x40 0x64)",
        .argtable = &tmc_write_args,
        .func     = cmd_tmc_write,
    };
    esp_console_cmd_register(&tmc_write_cmd);

    // tmc_dump
    esp_console_cmd_t tmc_dump_cmd = {
        .command = "tmc_dump",
        .help    = "Dump all key TMC2209 registers",
        .func    = cmd_tmc_dump,
    };
    esp_console_cmd_register(&tmc_dump_cmd);

    // sg_monitor <steps> [fwd|rev]
    sg_mon_args.count = arg_int1(NULL, NULL, "<steps>", "Number of microsteps to run");
    sg_mon_args.dir   = arg_str0(NULL, NULL, "[fwd|rev]", "Direction (default: fwd)");
    sg_mon_args.end   = arg_end(2);
    esp_console_cmd_t sg_mon_cmd = {
        .command  = "sg_monitor",
        .help     = "Move N steps and print SG_RESULT every 100ms (CSV: sample,SG_RESULT,pos)",
        .argtable = &sg_mon_args,
        .func     = cmd_sg_monitor,
    };
    esp_console_cmd_register(&sg_mon_cmd);

    // sg_config <threshold> [tcoolthrs]
    sg_args.threshold = arg_int1(NULL, NULL, "<threshold>", "SGTHRS 0-255 (higher=more sensitive)");
    sg_args.tcoolthrs = arg_int0(NULL, NULL, "[tcoolthrs]", "TCOOLTHRS velocity threshold");
    sg_args.end       = arg_end(2);
    esp_console_cmd_t sg_cmd = {
        .command  = "sg_config",
        .help     = "Configure StallGuard threshold",
        .argtable = &sg_args,
        .func     = cmd_sg_config,
    };
    esp_console_cmd_register(&sg_cmd);

    // zb_reset
    esp_console_cmd_t zb_reset_cmd = {
        .command = "zb_reset",
        .help    = "Factory-reset Zigbee credentials and restart pairing",
        .func    = cmd_zb_reset,
    };
    esp_console_cmd_register(&zb_reset_cmd);
}

// ============================================================================
// app_main
// ============================================================================
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));   // wait for USB Serial/JTAG to enumerate
    ESP_LOGI(TAG, "LockBee starting...");

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Initialize motor
    lock_motor_config_t cfg = make_default_config();
    esp_err_t ret = lock_motor_init(&g_motor, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor init failed: %s", esp_err_to_name(ret));
        // Continue anyway so the console is accessible for debugging
    } else {
        ESP_LOGI(TAG, "Motor init OK");
        uint32_t saved_speed = nvs_load_speed(g_motor.cfg.step_period_us);
        if (saved_speed != g_motor.cfg.step_period_us) {
            lock_motor_set_speed(&g_motor, saved_speed);
            ESP_LOGI(TAG, "Speed restored from NVS: %"PRIu32" µs/step", saved_speed);
        }

        uint8_t saved_irun  = nvs_load_u8(NVS_KEY_IRUN,  g_motor.cfg.irun);
        uint8_t saved_ihold = nvs_load_u8(NVS_KEY_IHOLD, g_motor.cfg.ihold);
        if (saved_irun != g_motor.cfg.irun || saved_ihold != g_motor.cfg.ihold) {
            g_motor.cfg.irun  = saved_irun;
            g_motor.cfg.ihold = saved_ihold;
            tmc2209_set_current(&g_motor.tmc, saved_irun, saved_ihold,
                                g_motor.cfg.ihold_delay);
            ESP_LOGI(TAG, "Current restored from NVS: IRUN=%d IHOLD=%d",
                     saved_irun, saved_ihold);
        }

        uint32_t saved_lock_steps = nvs_load_u32(NVS_KEY_LOCK_STEPS, g_motor.cfg.lock_steps);
        if (saved_lock_steps != g_motor.cfg.lock_steps && saved_lock_steps > 0) {
            g_motor.cfg.lock_steps = saved_lock_steps;
            g_motor.unlocked_pos   = saved_lock_steps;
            ESP_LOGI(TAG, "Lock steps restored from NVS: %"PRIu32, saved_lock_steps);
        }

        tmc2209_dump_regs(&g_motor.tmc);
    }

    // Setup console (USB Serial/JTAG on ESP32-C6)
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt       = "lockbee> ";
    repl_cfg.max_cmdline_length = 128;

    esp_console_register_help_command();
    register_commands();

#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t usb_cfg =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ret = esp_console_new_repl_usb_serial_jtag(&usb_cfg, &repl_cfg, &repl);
#else
    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ret = esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl);
#endif

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Console init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize Zigbee (starts Zigbee + motor-command tasks)
    ret = zigbee_init(&g_motor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Zigbee init failed: %s", esp_err_to_name(ret));
        // Continue anyway — console still usable for diagnostics
    }

    // Initialize TTP223 touch buttons (open=GPIO5, close=GPIO6)
    ret = touch_init(&g_motor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Touch init failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Console ready. Type 'help' for commands.");
    esp_console_start_repl(repl);
}
