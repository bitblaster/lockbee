#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------
#define TMC2209_REG_GCONF       0x00  // RW: General configuration
#define TMC2209_REG_GSTAT       0x01  // RWC: Global status
#define TMC2209_REG_IFCNT       0x02  // R:  UART transmission counter
#define TMC2209_REG_SLAVECONF   0x03  // W:  Slave configuration / send delay
#define TMC2209_REG_IOIN        0x06  // R:  Input pin states
#define TMC2209_REG_IHOLD_IRUN  0x10  // W:  Current control
#define TMC2209_REG_TPOWERDOWN  0x11  // W:  Power-down delay
#define TMC2209_REG_TSTEP       0x12  // R:  Actual microstep time
#define TMC2209_REG_TPWMTHRS    0x13  // W:  StealthChop upper velocity threshold
#define TMC2209_REG_TCOOLTHRS   0x14  // W:  StallGuard lower velocity threshold
#define TMC2209_REG_VACTUAL     0x22  // W:  Direct velocity control
#define TMC2209_REG_SGTHRS      0x40  // W:  StallGuard threshold
#define TMC2209_REG_SG_RESULT   0x41  // R:  StallGuard result (0 = stall)
#define TMC2209_REG_COOLCONF    0x42  // W:  CoolStep configuration
#define TMC2209_REG_MSCNT       0x6A  // R:  Microstep counter
#define TMC2209_REG_MSCURACT    0x6B  // R:  Actual microstep current
#define TMC2209_REG_CHOPCONF    0x6C  // RW: Chopper configuration
#define TMC2209_REG_DRV_STATUS  0x6F  // R:  Driver status flags
#define TMC2209_REG_PWMCONF     0x70  // RW: StealthChop configuration
#define TMC2209_REG_PWM_SCALE   0x71  // R:  StealthChop scale
#define TMC2209_REG_PWM_AUTO    0x72  // R:  StealthChop auto-tuning result

// ---------------------------------------------------------------------------
// GCONF bit fields
// ---------------------------------------------------------------------------
#define TMC2209_GCONF_I_SCALE_ANALOG    (1u << 0)
#define TMC2209_GCONF_INTERNAL_RSENSE   (1u << 1)
#define TMC2209_GCONF_EN_SPREADCYCLE    (1u << 2)  // 0=StealthChop, 1=SpreadCycle
#define TMC2209_GCONF_SHAFT             (1u << 3)  // invert motor direction
#define TMC2209_GCONF_INDEX_OTPW        (1u << 4)
#define TMC2209_GCONF_INDEX_STEP        (1u << 5)
#define TMC2209_GCONF_PDN_DISABLE       (1u << 6)  // must set 1 to use UART
#define TMC2209_GCONF_MSTEP_REG_SELECT  (1u << 7)  // 1 = use MRES from CHOPCONF
#define TMC2209_GCONF_MULTISTEP_FILT    (1u << 8)

// ---------------------------------------------------------------------------
// CHOPCONF bit fields (microstep resolution at bits 27:24)
// ---------------------------------------------------------------------------
#define TMC2209_CHOPCONF_MRES_SHIFT     24
#define TMC2209_CHOPCONF_MRES_MASK      (0xFu << 24)
#define TMC2209_CHOPCONF_INTPOL         (1u << 28)  // interpolation to 256

// MRES encoding: 0->256, 1->128, 2->64, 3->32, 4->16, 5->8, 6->4, 7->2, 8->1
static inline uint8_t tmc2209_mres_encode(uint16_t microsteps)
{
    switch (microsteps) {
        case 256: return 0;
        case 128: return 1;
        case  64: return 2;
        case  32: return 3;
        case  16: return 4;
        case   8: return 5;
        case   4: return 6;
        case   2: return 7;
        default:  return 8; // 1 (full step)
    }
}

// ---------------------------------------------------------------------------
// DRV_STATUS bit fields
// ---------------------------------------------------------------------------
#define TMC2209_DRV_STATUS_OT     (1u << 1)   // overtemperature shutdown
#define TMC2209_DRV_STATUS_OTPW   (1u << 0)   // overtemperature warning
#define TMC2209_DRV_STATUS_S2GA   (1u << 2)   // short to GND phase A
#define TMC2209_DRV_STATUS_S2GB   (1u << 3)   // short to GND phase B
#define TMC2209_DRV_STATUS_CS_ACT_SHIFT 16
#define TMC2209_DRV_STATUS_CS_ACT_MASK (0x1Fu << 16)
#define TMC2209_DRV_STATUS_STST   (1u << 31)  // standstill indicator

// ---------------------------------------------------------------------------
// Driver handle
// ---------------------------------------------------------------------------
typedef struct {
    int      uart_port;   // UART peripheral number
    int      tx_gpio;
    int      rx_gpio;
    uint8_t  slave_addr;  // 0-3
} tmc2209_config_t;

typedef struct {
    tmc2209_config_t cfg;
    bool             initialized;
} tmc2209_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

/**
 * @brief Initialize TMC2209 UART interface (does NOT configure motor registers).
 */
esp_err_t tmc2209_init(tmc2209_t *dev, const tmc2209_config_t *cfg);

/**
 * @brief Write a 32-bit value to a TMC2209 register via UART.
 */
esp_err_t tmc2209_write_reg(tmc2209_t *dev, uint8_t reg, uint32_t value);

/**
 * @brief Read a 32-bit value from a TMC2209 register via UART.
 */
esp_err_t tmc2209_read_reg(tmc2209_t *dev, uint8_t reg, uint32_t *value);

/**
 * @brief Configure motor run and hold currents.
 *
 * @param run_scale   IRUN  0-31
 * @param hold_scale  IHOLD 0-31
 * @param hold_delay  IHOLDDELAY 0-15
 */
esp_err_t tmc2209_set_current(tmc2209_t *dev,
                               uint8_t run_scale,
                               uint8_t hold_scale,
                               uint8_t hold_delay);

/**
 * @brief Set microstep resolution via CHOPCONF (requires MSTEP_REG_SELECT=1 in GCONF).
 *
 * @param microsteps  1, 2, 4, 8, 16, 32, 64, 128, or 256
 */
esp_err_t tmc2209_set_microsteps(tmc2209_t *dev, uint16_t microsteps);

/**
 * @brief Configure StallGuard2 sensitivity.
 *
 * @param threshold   SGTHRS 0-255 (higher = more sensitive)
 * @param tcoolthrs   Lower velocity threshold for StallGuard activation
 */
esp_err_t tmc2209_set_stallguard(tmc2209_t *dev, uint8_t threshold, uint32_t tcoolthrs);

/**
 * @brief Read current StallGuard result (0 = stalled, 510 = no load).
 */
esp_err_t tmc2209_get_sg_result(tmc2209_t *dev, uint16_t *sg_result);

/**
 * @brief Read DRV_STATUS register.
 */
esp_err_t tmc2209_get_drv_status(tmc2209_t *dev, uint32_t *status);

/**
 * @brief Switch between StealthChop (false) and SpreadCycle (true) modes.
 *        SpreadCycle gives more torque, StealthChop is quieter.
 */
esp_err_t tmc2209_set_spreadcycle(tmc2209_t *dev, bool enable);

/**
 * @brief Read IFCNT counter to verify UART write was received.
 */
esp_err_t tmc2209_get_ifcnt(tmc2209_t *dev, uint8_t *count);

/**
 * @brief Print a human-readable dump of key registers (uses ESP_LOG).
 */
void tmc2209_dump_regs(tmc2209_t *dev);
