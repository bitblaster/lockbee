# LockBee

ESP32-H2 based smart lock actuator using a NEMA17 stepper motor and TMC2209 driver.
The crown gear is mounted in-axis with the lock cylinder thumb-turn; the motor sits
offset below, connected via a 3D-printed gear reduction. External key operation is
left intact.

---

## Hardware

| Component | Model | Notes |
|---|---|---|
| MCU | ESP32-H2 SuperMini | Zigbee + BLE, USB CDC console |
| Stepper motor | 42BYGH34 dual-shaft | NEMA17, 1.8°/step, 200 steps/rev |
| Motor driver | TMC2209 module | UART single-wire, StallGuard4 |
| Power supply | 14V DC | Powers motor directly |
| Buck converter | Mini 560 | 14V → 5V; ESP32 onboard LDO regulates to 3.3V |

### Gear reduction

The lock required more torque than the motor could deliver at IRUN=31.
A 3D-printed gear pair (PETG) was designed to multiply torque by ~3.85×.
The crown gear is coaxial with the lock thumb-turn; the motor sits offset below.

| Parameter | Motor pinion | Lock crown |
|---|:---:|:---:|
| Module | 1 | 1 |
| Teeth | 13 | 50 |
| Pitch Ø | 13 mm | 50 mm |
| Tip Ø | 15 mm | 52 mm |
| Face width | 10 mm | 10 mm |
| Hub | Ø14 × 15 mm, bore 5 mm, M3 grub screw | bore = lock shaft |

Center distance: **32 mm** (theoretical 31.5 mm) — Ratio: **3.85:1**

The pinion mounts on the NEMA17 5 mm shaft (D-flat side for the M3 grub screw).
The crown gear mounts on the lock thumb-turn shaft.

---

## Wiring

All six signals are routed to the right-side pins of the SuperMini (GP9–GP14),
keeping the left side free for future use or clean PCB routing.

| Signal | ESP32-H2 GPIO | SuperMini pin | Target |
|---|:---:|:---:|---|
| STEP | 9 | G9 | TMC2209 STEP |
| DIR | 10 | G10 | TMC2209 DIR |
| EN (active LOW) | 11 | G11 | TMC2209 EN |
| UART TX | 12 | G12 | TMC2209 PDN_UART |
| UART RX | 13 | G13 | TMC2209 PDN_UART (via 1 kΩ) |
| DIAG | 14 | G14 | TMC2209 DIAG |
| Touch OPEN | 5 | G5 | TTP223 module #1 output |
| Touch CLOSE | 6 | G6 | TTP223 module #2 output |

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
14V DC ──┬──────────────────────────── TMC2209 VM (motor power)
         │
         └── Mini 560 buck ── 5V ──── ESP32-H2 5V pin
                                  └── TMC2209 VIO
                                        (ESP32 onboard LDO → 3.3V internally)
```
The buck converter outputs 5V rather than 3.3V to reduce output ripple through
the ESP32's onboard LDO before reaching sensitive logic circuits.

---

## Firmware Architecture

```
main/
├── main.c            — app_main, console REPL, CLI commands, NVS persistence
├── tmc2209.h/.c      — TMC2209 UART driver (CRC-8, register R/W, StallGuard)
├── lock_motor.h/.c   — GPTimer step generation, lock open/close/calibrate
├── zigbee.h/.c       — Zigbee stack (Door Lock cluster), ZCL callbacks, motor queue
├── touch.h/.c        — TTP223 touch button handler (ISR + debounce task)
└── Kconfig.projbuild — menuconfig options for all GPIOs and motor parameters
```

### Step generation

A **GPTimer** running at 1 MHz generates STEP pulses via ISR callback.
Each callback fires at `step_period_us` intervals, toggles the STEP GPIO,
and decrements the step counter. The ISR checks the `stall_detected` flag
set by the DIAG interrupt and stops the motor immediately on stall.

### StallGuard4 end-stop detection

The lock has no physical limit switches — end-stops are detected via TMC2209
StallGuard4: when motor load exceeds the threshold, SG_RESULT drops and DIAG
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

### Task architecture

All motor commands — from Zigbee, the console REPL, and touch buttons — are
serialised through a single FreeRTOS queue consumed by `motor_task`:

```
zb_task (ZB stack loop, prio 5)
  └── ZCL callbacks → xQueueSend(motor_cmd_queue)

motor_task (prio 4)
  └── xQueueReceive → lock_motor_open/close() [blocking]
        └── esp_zb_scheduler_user_alarm(update_lock_state_cb, 0)

Console REPL task
  └── cmd_open/cmd_close → lock_motor_open/close() [blocking]
        └── zigbee_schedule_lock_state_update()

touch_task (prio 3)
  └── GPIO ISR → queue → debounce → zigbee_motor_cmd_send()
        └── xQueueSend(motor_cmd_queue)
```

ZCL attribute updates (`esp_zb_zcl_set_attribute_val`) must run inside the
Zigbee task context; `motor_task` posts them back via
`esp_zb_scheduler_user_alarm(cb, NULL, 0)` (delay 0 = next Zigbee stack tick).

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
| `zb_reset` | Factory-reset Zigbee (clears stored network, triggers re-pairing) |

---

## Zigbee / Home Assistant

The firmware exposes a **Zigbee Door Lock** device (ZCL cluster 0x0101),
compatible with Zigbee2MQTT and Home Assistant out of the box.

### Commissioning (first pairing)

1. Flash the firmware and power on. The device immediately starts scanning for
   a Zigbee coordinator (network steering). If no coordinator is found it
   retries every second — no manual action needed.
2. In Zigbee2MQTT: open **Permit join** (global or for a specific router).
3. The device joins the network and appears as a `lock` entity in Home
   Assistant. No YAML configuration is required — the Door Lock cluster is
   auto-discovered.

To re-pair (e.g. after replacing the coordinator): run `zb_reset` from the
USB console. The device clears its stored network credentials and restarts
the pairing scan.

### Supported ZCL commands

| ZCL command | ZCL code | Firmware action |
|---|:---:|---|
| Lock Door | 0x00 | Motor moves to locked position |
| Unlock Door | 0x01 | Motor moves to unlocked position |
| Unlock with Timeout | 0x03 | Motor unlocks, then re-locks after N seconds |

**UnlockWithTimeout** is handled entirely in firmware — no automation on the
HA side is required. The timeout (uint16, seconds) is encoded in the ZCL
payload. If a new command arrives before the timer expires, the pending
re-lock is cancelled.

### LockState attribute reporting

After every motor move the firmware updates the ZCL `LockState` attribute
(cluster 0x0101, attribute 0x0000) and pushes an unsolicited report to the
coordinator:

| `LockState` value | Meaning |
|:---:|---|
| 0 | Not fully locked (transitioning or unknown) |
| 1 | Locked |
| 2 | Unlocked |

Home Assistant reflects the new state in real-time without polling.

### Home Assistant automation examples

**Auto-lock after the door closes (reed switch on a separate sensor):**
```yaml
automation:
  - alias: "Auto-lock after door closes"
    trigger:
      - platform: state
        entity_id: binary_sensor.door_reed
        to: "off"          # reed closed = door shut
    condition:
      - condition: state
        entity_id: lock.lockbee
        state: "unlocked"
    action:
      - delay: "00:00:30"
      - service: lock.lock
        target: {entity_id: lock.lockbee}
```

**Push notification on every lock state change:**
```yaml
automation:
  - alias: "LockBee state notification"
    trigger:
      - platform: state
        entity_id: lock.lockbee
    action:
      - service: notify.mobile_app_YOURPHONE
        data:
          title: "LockBee"
          message: "Serratura: {{ states('lock.lockbee') | upper }}"
```

---

## Touch Buttons

Two **TTP223** capacitive touch modules allow direct local control without
any network dependency:

| Module | GPIO | Action |
|---|:---:|---|
| Touch OPEN | G5 | Unlocks the door (same as `open` console command) |
| Touch CLOSE | G6 | Locks the door (same as `close` console command) |

### Wiring

Power the TTP223 modules from the ESP32 **3.3 V** rail (not 5 V). The module
output is **push-pull active HIGH** — connect directly to the GPIO, no pull
resistor needed:

```
ESP32 3V3 ──── TTP223 VCC
ESP32 GND ──── TTP223 GND
ESP32 G5  ──── TTP223 #1 OUT  (OPEN)
ESP32 G6  ──── TTP223 #2 OUT  (CLOSE)
```

### Firmware behaviour

- A **rising-edge GPIO ISR** fires on each touch and posts the button index
  to a FreeRTOS queue.
- `touch_task` reads the queue and calls `zigbee_motor_cmd_send()`, posting
  to the shared motor command queue — the same queue used by Zigbee and
  console commands. All three sources are fully serialised; concurrent
  presses never cause a race condition.
- No software debounce is needed: the TTP223 chip integrates hardware
  debounce and outputs a clean digital signal.
- After the motor move completes, the `LockState` ZCL attribute is updated
  just as it would be for any other command source.

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
**Solution:** 3D-printed gear reduction (3.85:1, z13/z50, module 1) with the crown
coaxial to the lock thumb-turn and the motor offset below. Printed in PETG.

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
- 3D-printed enclosure with dedicated slots for ESP32, TMC2209 module, and buck converter
- ZCL PIN code management (SetPINCode cluster commands) for keypad integration
