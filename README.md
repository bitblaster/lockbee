# LockBee

ESP32-H2 based smart lock actuator using a NEMA17 stepper motor and TMC2209 driver.
The motor is mounted in-axis with the lock cylinder via a 3D-printed gear reduction,
replacing the internal thumb-turn knob while leaving the external key operation intact.

---

## Hardware

| Component | Model | Notes |
|---|---|---|
| MCU | ESP32-H2 SuperMini | Zigbee + BLE, USB CDC console |
| Stepper motor | 42BYGH34 dual-shaft | NEMA17, 1.8°/step, 200 steps/rev |
| Motor driver | TMC2209 module | UART single-wire, StallGuard2 |
| Power supply | 14V DC | Powers motor directly |
| Buck converter | Mini 560 | 14V → 3.3V for ESP32 |

### Gear reduction

The lock required more torque than the motor could deliver at IRUN=31.
A 3D-printed gear pair (PETG) was designed to multiply torque by ~2.54×:

| Parameter | Motor pinion | Lock crown |
|---|:---:|:---:|
| Module | 1 | 1 |
| Teeth | 13 | 33 |
| Pitch Ø | 13 mm | 33 mm |
| Tip Ø | 15 mm | 35 mm |
| Face width | 10 mm | 10 mm |
| Hub | Ø14 × 15 mm, bore 5 mm, M3 grub screw | bore = lock shaft |

Center distance: **23 mm** — Ratio: **2.54:1**

The pinion mounts on the NEMA17 5 mm shaft (D-flat side for the M3 grub screw).
The crown gear mounts on the lock thumb-turn shaft.

---

## Wiring

All six signals are routed to the right-side pins of the SuperMini (GP9–GP14),
keeping the left side free for future use or clean PCB routing.

| Signal | ESP32-H2 GPIO | SuperMini pin | TMC2209 pin |
|---|:---:|:---:|---|
| STEP | 9 | G9 | STEP |
| DIR | 10 | G10 | DIR |
| EN (active LOW) | 11 | G11 | EN |
| UART TX | 12 | G12 | PDN_UART |
| UART RX | 13 | G13 | PDN_UART (via 1 kΩ) |
| DIAG | 14 | G14 | DIAG |

> **GPIO 9** doubles as the BOOT strapping pin, but causes no conflict because
> STEP is driven by the ESP32 (output) into the TMC2209 high-impedance input —
> nothing pulls it LOW at boot.

### UART single-wire connection

The TMC2209 uses a single-wire half-duplex UART on PDN_UART.
TX is connected directly to PDN_UART; RX is looped back through a **1 kΩ** resistor:

```
ESP32 TX (G12) ──────────┬──── TMC2209 PDN_UART
                         │
ESP32 RX (G13) ──[1kΩ]──┘
```

The firmware flushes the 4-byte TX echo before reading the 8-byte reply.

### DIAG

The TMC2209 DIAG output is **push-pull** (per datasheet): no pull resistor required.
- **Normal operation**: chip actively drives DIAG LOW
- **Stall detected**: chip drives DIAG HIGH → rising-edge interrupt on ESP32

### Power

```
14V DC ──┬──────────────────── TMC2209 VM (motor power)
         │
         └── Mini 560 buck ─── 3.3V ── ESP32-H2 3V3 pin
                                    └── TMC2209 VIO
```

---

## Firmware Architecture

```
main/
├── main.c           — app_main, console REPL, CLI commands, NVS persistence
├── tmc2209.h/.c     — TMC2209 UART driver (CRC-8, register R/W, StallGuard)
├── lock_motor.h/.c  — GPTimer step generation, lock open/close/calibrate
└── Kconfig.projbuild — menuconfig options for all GPIOs and motor parameters
```

### Step generation

A **GPTimer** running at 1 MHz generates STEP pulses via ISR callback.
Each callback fires at `step_period_us` intervals, toggles the STEP GPIO,
and decrements the step counter. The ISR checks the `stall_detected` flag
set by the DIAG interrupt and stops the motor immediately on stall.

### StallGuard2 end-stop detection

The lock has no physical limit switches — end-stops are detected via TMC2209
StallGuard2: when motor load exceeds the threshold, SG_RESULT drops and DIAG
is asserted, triggering a GPIO rising-edge interrupt that stops the motor.

**Critical requirement (from datasheet §5.2):**
StallGuard on DIAG is only active in **StealthChop mode** and only when:
```
TCOOLTHRS ≥ TSTEP > TPWMTHRS
```
TPWMTHRS is set to 0 (no upper speed limit). TCOOLTHRS must satisfy:
```
TCOOLTHRS > step_period_µs × fCLK_MHz × safety_margin
           = 100 × 12 × 5 = 6000   (at default speed)
```

### Calibration

`lock_motor_calibrate()` drives to both mechanical end-stops using StallGuard,
measures the actual stroke in microsteps, and saves it to NVS. On reboot,
the calibrated value is restored — no re-calibration needed after power cycle.

### NVS persistence

| Key | Type | Content |
|---|---|---|
| `speed` | u32 | Step period in µs |
| `irun` | u8 | TMC2209 run current scale |
| `ihold` | u8 | TMC2209 hold current scale |
| `lock_steps` | u32 | Calibrated stroke in microsteps |

---

## Calibrated Parameters

Measured on the actual lock with 2.54:1 gear reduction:

| Parameter | Value |
|---|---|
| Full stroke | **36 000 microsteps** |
| Step period (`speed`) | **100 µs** |
| StallGuard threshold (`SGTHRS`) | **100** |
| `TCOOLTHRS` | **6 000** |
| Microstep resolution | 16 |
| IRUN | 31 |

---

## Console Commands (USB CDC)

Connect at 115200 baud (or use `idf.py monitor`).

| Command | Description |
|---|---|
| `open` | Move to unlocked position (StallGuard end-stop) |
| `close` | Move to locked position (StallGuard end-stop) |
| `stop` | Stop motor immediately |
| `calibrate` | Measure stroke: FWD to locked, REV to unlocked, save to NVS |
| `step <n> [fwd\|rev]` | Move N microsteps |
| `speed <us>` | Set step period in µs (saved to NVS) |
| `enable <on\|off>` | Enable/disable TMC2209 EN pin |
| `status` | Motor state, position, DRV_STATUS, SG_RESULT |
| `sg_config <sgthrs> [tcoolthrs]` | Set StallGuard threshold and velocity window |
| `sg_monitor <n> [fwd\|rev]` | Move N steps, print SG_RESULT every 100 ms (CSV) |
| `tmc_read <reg>` | Read TMC2209 register (hex, e.g. `0x6F`) |
| `tmc_write <reg> <val>` | Write TMC2209 register |
| `tmc_dump` | Dump all key TMC2209 registers |

---

## Build

```bash
# Activate ESP-IDF 5.4.2
source ~/.espressif/python_env/idf5.4_py3.12_env/bin/activate
export IDF_PATH=/opt/esp32/esp-idf-v5.4.2
export PATH="$HOME/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin:$PATH"

# First build (or after changing GPIOs): delete cached config
rm -f sdkconfig

# Build and flash
python3 $IDF_PATH/tools/idf.py build
python3 $IDF_PATH/tools/idf.py -p /dev/ttyACM0 flash monitor
```

---

## Problems Solved

### 1. Insufficient torque
The NEMA17 alone could not turn the lock cylinder even at maximum current (IRUN=31).
**Solution:** 3D-printed gear reduction (2.54:1) designed to fit within the 35 mm
diameter constraint of the lock housing. Printed in PETG.

### 2. StallGuard not triggering — DIAG always LOW
After connecting the DIAG pin and configuring a rising-edge interrupt, the DIAG signal
stayed at 0 V regardless of `SGTHRS` and `TCOOLTHRS` values.

**Root cause:** **SpreadCycle was enabled** (`GCONF.en_SpreadCycle = 1`).
The TMC2209 datasheet (§5.2) states: *"DIAG is pulsed by StallGuard... It is only
enabled in StealthChop mode."* StallGuard on DIAG — and SG_RESULT register updates —
are completely disabled in SpreadCycle mode. Fixed by removing
`tmc2209_set_spreadcycle(true)` from the initialization sequence and setting
`TPWMTHRS = 0`. DIAG is a push-pull output per datasheet — no pull resistor needed.

**Diagnostic tool added:** `sg_monitor` command starts a move asynchronously and
polls `SG_RESULT` via UART every 100 ms, printing CSV output — allowing real-time
StallGuard observation without blocking the console.

### 3. TCOOLTHRS values too small
Initial tests used `TCOOLTHRS = 50` and `TCOOLTHRS = 400`. At the operating speed
of 100 µs/step, `TSTEP ≈ 1200` clock ticks. StallGuard only activates when
`TCOOLTHRS ≥ TSTEP`, so values below 1200 disable it entirely.
**Solution:** `TCOOLTHRS = 6000` (5× safety margin over TSTEP).

### 4. No .gitignore — build artifacts staged
The ESP-IDF `build/` directory contains hundreds of compiled objects and binaries.
**Solution:** `.gitignore` added to exclude `build/`, `sdkconfig`, and `sdkconfig.old`.

---

## Next Steps

- Acceleration ramp in `lock_motor.c` for smoother start/stop
- Zigbee integration for Home Assistant
- 3D-printed enclosure with dedicated slots for ESP32, TMC2209 module, and buck converter
