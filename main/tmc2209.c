#include "tmc2209.h"

#include <string.h>
#include "driver/uart.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "tmc2209";

// ---------------------------------------------------------------------------
// UART datagram sizes
// ---------------------------------------------------------------------------
#define WRITE_DATAGRAM_LEN  8
#define READ_REQ_LEN        4
#define READ_REPLY_LEN      8
#define UART_TIMEOUT_MS     20

// ---------------------------------------------------------------------------
// CRC-8 (polynomial 0x07) as specified by TMC2209 datasheet
// ---------------------------------------------------------------------------
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            byte >>= 1;
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
esp_err_t tmc2209_init(tmc2209_t *dev, const tmc2209_config_t *cfg)
{
    if (!dev || !cfg) return ESP_ERR_INVALID_ARG;

    dev->cfg = *cfg;
    dev->initialized = false;

    uart_config_t uart_cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_param_config(cfg->uart_port, &uart_cfg),
                        TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(cfg->uart_port,
                                     cfg->tx_gpio, cfg->rx_gpio,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");
    ESP_RETURN_ON_ERROR(uart_driver_install(cfg->uart_port, 256, 256, 0, NULL, 0),
                        TAG, "uart_driver_install failed");

    dev->initialized = true;
    ESP_LOGI(TAG, "UART%d initialized (TX=%d RX=%d slave=0x%02x)",
             cfg->uart_port, cfg->tx_gpio, cfg->rx_gpio, cfg->slave_addr);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Write register
// UART single-wire: we transmit 8 bytes and our TX is looped back into RX,
// so we flush the echo right after transmitting.
// ---------------------------------------------------------------------------
esp_err_t tmc2209_write_reg(tmc2209_t *dev, uint8_t reg, uint32_t value)
{
    if (!dev || !dev->initialized) return ESP_ERR_INVALID_STATE;

    uint8_t buf[WRITE_DATAGRAM_LEN];
    buf[0] = 0x05;                          // sync byte
    buf[1] = dev->cfg.slave_addr;           // node address
    buf[2] = reg | 0x80;                    // register + write flag
    buf[3] = (value >> 24) & 0xFF;
    buf[4] = (value >> 16) & 0xFF;
    buf[5] = (value >>  8) & 0xFF;
    buf[6] = (value >>  0) & 0xFF;
    buf[7] = crc8(buf, 7);

    // Flush any stale bytes in RX buffer
    uart_flush(dev->cfg.uart_port);

    uart_write_bytes(dev->cfg.uart_port, buf, WRITE_DATAGRAM_LEN);

    // Wait for TX to complete, then flush our own echo from RX
    uart_wait_tx_done(dev->cfg.uart_port, pdMS_TO_TICKS(UART_TIMEOUT_MS));
    uart_flush_input(dev->cfg.uart_port);

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Read register
// Send 4-byte read request, discard 4-byte echo, read 8-byte reply.
// ---------------------------------------------------------------------------
esp_err_t tmc2209_read_reg(tmc2209_t *dev, uint8_t reg, uint32_t *value)
{
    if (!dev || !dev->initialized || !value) return ESP_ERR_INVALID_ARG;

    uint8_t req[READ_REQ_LEN];
    req[0] = 0x05;
    req[1] = dev->cfg.slave_addr;
    req[2] = reg & 0x7F;                    // read flag: MSB = 0
    req[3] = crc8(req, 3);

    uart_flush(dev->cfg.uart_port);
    uart_write_bytes(dev->cfg.uart_port, req, READ_REQ_LEN);
    uart_wait_tx_done(dev->cfg.uart_port, pdMS_TO_TICKS(UART_TIMEOUT_MS));

    // Discard echo of our own request
    uint8_t echo[READ_REQ_LEN];
    int n = uart_read_bytes(dev->cfg.uart_port, echo, READ_REQ_LEN,
                            pdMS_TO_TICKS(UART_TIMEOUT_MS));
    if (n != READ_REQ_LEN) {
        ESP_LOGW(TAG, "read_reg 0x%02x: echo timeout (got %d bytes)", reg, n);
        return ESP_ERR_TIMEOUT;
    }

    // Read the actual reply
    uint8_t reply[READ_REPLY_LEN];
    n = uart_read_bytes(dev->cfg.uart_port, reply, READ_REPLY_LEN,
                        pdMS_TO_TICKS(UART_TIMEOUT_MS));
    if (n != READ_REPLY_LEN) {
        ESP_LOGW(TAG, "read_reg 0x%02x: reply timeout (got %d bytes)", reg, n);
        return ESP_ERR_TIMEOUT;
    }

    // Validate CRC
    uint8_t expected_crc = crc8(reply, READ_REPLY_LEN - 1);
    if (reply[READ_REPLY_LEN - 1] != expected_crc) {
        ESP_LOGW(TAG, "read_reg 0x%02x: CRC mismatch (got 0x%02x, expected 0x%02x)",
                 reg, reply[READ_REPLY_LEN - 1], expected_crc);
        return ESP_ERR_INVALID_CRC;
    }

    // reply[0]=0x05, reply[1]=0xFF (master addr), reply[2]=reg, reply[3..6]=data
    *value = ((uint32_t)reply[3] << 24)
           | ((uint32_t)reply[4] << 16)
           | ((uint32_t)reply[5] <<  8)
           | ((uint32_t)reply[6] <<  0);

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Current control
// ---------------------------------------------------------------------------
esp_err_t tmc2209_set_current(tmc2209_t *dev,
                               uint8_t run_scale,
                               uint8_t hold_scale,
                               uint8_t hold_delay)
{
    if (run_scale  > 31) run_scale  = 31;
    if (hold_scale > 31) hold_scale = 31;
    if (hold_delay > 15) hold_delay = 15;

    uint32_t val = ((uint32_t)hold_delay << 16)
                 | ((uint32_t)run_scale  <<  8)
                 | ((uint32_t)hold_scale <<  0);

    ESP_LOGI(TAG, "IHOLD_IRUN: IRUN=%u IHOLD=%u IHOLDDELAY=%u",
             run_scale, hold_scale, hold_delay);
    return tmc2209_write_reg(dev, TMC2209_REG_IHOLD_IRUN, val);
}

// ---------------------------------------------------------------------------
// Microstep resolution
// ---------------------------------------------------------------------------
esp_err_t tmc2209_set_microsteps(tmc2209_t *dev, uint16_t microsteps)
{
    uint32_t chopconf;
    esp_err_t ret = tmc2209_read_reg(dev, TMC2209_REG_CHOPCONF, &chopconf);
    if (ret != ESP_OK) {
        // Can't read back; start from a safe default (spread cycle, toff=3)
        chopconf = 0x10000053;
        ESP_LOGW(TAG, "set_microsteps: using default CHOPCONF 0x%08"PRIx32, chopconf);
    }

    uint8_t mres = tmc2209_mres_encode(microsteps);
    chopconf &= ~TMC2209_CHOPCONF_MRES_MASK;
    chopconf |= ((uint32_t)mres << TMC2209_CHOPCONF_MRES_SHIFT);
    chopconf |= TMC2209_CHOPCONF_INTPOL; // always enable interpolation to 256

    ESP_LOGI(TAG, "CHOPCONF: microsteps=%u MRES=%u", microsteps, mres);
    return tmc2209_write_reg(dev, TMC2209_REG_CHOPCONF, chopconf);
}

// ---------------------------------------------------------------------------
// StallGuard2
// ---------------------------------------------------------------------------
esp_err_t tmc2209_set_stallguard(tmc2209_t *dev, uint8_t threshold, uint32_t tcoolthrs)
{
    esp_err_t ret;

    ret = tmc2209_write_reg(dev, TMC2209_REG_SGTHRS, threshold);
    if (ret != ESP_OK) return ret;

    ret = tmc2209_write_reg(dev, TMC2209_REG_TCOOLTHRS, tcoolthrs);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "StallGuard: SGTHRS=%u TCOOLTHRS=%"PRIu32, threshold, tcoolthrs);
    return ESP_OK;
}

esp_err_t tmc2209_get_sg_result(tmc2209_t *dev, uint16_t *sg_result)
{
    uint32_t val;
    esp_err_t ret = tmc2209_read_reg(dev, TMC2209_REG_SG_RESULT, &val);
    if (ret == ESP_OK) {
        *sg_result = (uint16_t)(val & 0x1FF); // 9 bits
    }
    return ret;
}

// ---------------------------------------------------------------------------
// DRV_STATUS
// ---------------------------------------------------------------------------
esp_err_t tmc2209_get_drv_status(tmc2209_t *dev, uint32_t *status)
{
    return tmc2209_read_reg(dev, TMC2209_REG_DRV_STATUS, status);
}

// ---------------------------------------------------------------------------
// SpreadCycle vs StealthChop
// ---------------------------------------------------------------------------
esp_err_t tmc2209_set_spreadcycle(tmc2209_t *dev, bool enable)
{
    uint32_t gconf;
    esp_err_t ret = tmc2209_read_reg(dev, TMC2209_REG_GCONF, &gconf);
    if (ret != ESP_OK) {
        gconf = 0;
        ESP_LOGW(TAG, "set_spreadcycle: GCONF read failed, starting from 0");
    }

    if (enable) {
        gconf |= TMC2209_GCONF_EN_SPREADCYCLE;
    } else {
        gconf &= ~TMC2209_GCONF_EN_SPREADCYCLE;
    }

    ESP_LOGI(TAG, "Mode: %s", enable ? "SpreadCycle" : "StealthChop");
    return tmc2209_write_reg(dev, TMC2209_REG_GCONF, gconf);
}

// ---------------------------------------------------------------------------
// IFCNT
// ---------------------------------------------------------------------------
esp_err_t tmc2209_get_ifcnt(tmc2209_t *dev, uint8_t *count)
{
    uint32_t val;
    esp_err_t ret = tmc2209_read_reg(dev, TMC2209_REG_IFCNT, &val);
    if (ret == ESP_OK) {
        *count = (uint8_t)(val & 0xFF);
    }
    return ret;
}

// ---------------------------------------------------------------------------
// Debug dump
// ---------------------------------------------------------------------------
void tmc2209_dump_regs(tmc2209_t *dev)
{
    uint32_t val;
    ESP_LOGI(TAG, "=== TMC2209 Register Dump ===");

    struct { uint8_t addr; const char *name; } regs[] = {
        { TMC2209_REG_GCONF,      "GCONF      " },
        { TMC2209_REG_GSTAT,      "GSTAT      " },
        { TMC2209_REG_IFCNT,      "IFCNT      " },
        { TMC2209_REG_IOIN,       "IOIN       " },
        { TMC2209_REG_IHOLD_IRUN, "IHOLD_IRUN " },
        { TMC2209_REG_TCOOLTHRS,  "TCOOLTHRS  " },
        { TMC2209_REG_TPWMTHRS,   "TPWMTHRS   " },
        { TMC2209_REG_SGTHRS,     "SGTHRS     " },
        { TMC2209_REG_SG_RESULT,  "SG_RESULT  " },
        { TMC2209_REG_CHOPCONF,   "CHOPCONF   " },
        { TMC2209_REG_DRV_STATUS, "DRV_STATUS " },
        { TMC2209_REG_PWMCONF,    "PWMCONF    " },
    };

    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        if (tmc2209_read_reg(dev, regs[i].addr, &val) == ESP_OK) {
            ESP_LOGI(TAG, "  0x%02x %s = 0x%08"PRIx32, regs[i].addr, regs[i].name, val);
        } else {
            ESP_LOGW(TAG, "  0x%02x %s = READ ERROR", regs[i].addr, regs[i].name);
        }
    }

    uint32_t drv_status;
    if (tmc2209_read_reg(dev, TMC2209_REG_DRV_STATUS, &drv_status) == ESP_OK) {
        ESP_LOGI(TAG, "  DRV_STATUS decoded:");
        ESP_LOGI(TAG, "    OT=%d  OTPW=%d  S2GA=%d  S2GB=%d  STST=%d  CS_ACTUAL=%"PRIu32,
            (int)!!(drv_status & TMC2209_DRV_STATUS_OT),
            (int)!!(drv_status & TMC2209_DRV_STATUS_OTPW),
            (int)!!(drv_status & TMC2209_DRV_STATUS_S2GA),
            (int)!!(drv_status & TMC2209_DRV_STATUS_S2GB),
            (int)!!(drv_status & TMC2209_DRV_STATUS_STST),
            (drv_status & TMC2209_DRV_STATUS_CS_ACT_MASK) >> TMC2209_DRV_STATUS_CS_ACT_SHIFT);
    }
}
