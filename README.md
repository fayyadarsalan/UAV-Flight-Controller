# UAV Flight Controller — ESP32-S3 (Full MAVLink Edition)

## Quick summary of changes in this version

| What changed | Old | New |
|---|---|---|
| Wing servos | 2 independent outputs | 1 output → Y-splitter → both servos |
| MAVLink layer | Hand-built protocol | Official MAVLink Arduino library |
| Waypoint upload | Broken (bad CRC_EXTRA by hand) | Fixed — uses official library handshake |
| STABILIZE mode | Decreased user inputs | **User commands angles directly (up to ±30° hard limit); plane levels when stick released** |
| AUTO mode | No user control | **User inputs blended 30/70 with autopilot** |
| Channel mapping | CH6: AUTO switch | **CH5: 3-pos (MANUAL/-100 / STABILIZE/0 / AUTO/+100); CH6: LOITER home** |
| ESC boot signal | Sent during init only | **Sent immediately at power-up, maintained in loop** |

---

## Hardware

| Role | Part |
|---|---|
| Flight controller | ESP32-S3 N16R8 |
| GPS | NEO-M8N-0-10 (u-blox) |
| IMU | MPU-6050 |
| Transmitter | FlySky FS-i6X |
| Receiver | FlySky FS-iA10B |
| Battery | 2200 mAh 3S LiPo |
| Motor | 1000 kV A2212 brushless + 1045R prop |
| Wing servos | 2× SG90S — both wired to one output via your Y-splitter |
| Elevator servo | 1× SG90S — independent output |
| ESC | Generic 30A (requires boot signal) |

---

## Required libraries (install via Arduino Library Manager)

| Library | Author | Why |
|---|---|---|
| **MAVLink** | MAVLink Development Team | Full protocol framing, CRC, message structs |
| **MPU6050_light** | rfetick | IMU reads and offset calibration |
| **TinyGPSPlus** | Mikal Hart | GPS sentence parsing |
| **ESP32Servo** | madhephaestus | Servo PWM on ESP32 |

`Preferences` (NVS parameter storage) is part of the ESP32 Arduino core — no separate install.

> If the `#include <mavlink.h>` line causes a "file not found" error, your library may use a
> dialect subfolder. Change it in `gcs.cpp` to `#include <common/mavlink.h>`.

---

## Wiring

### Y-splitter (wing ailerons)

- `AILERON_PIN` (GPIO 32) → Y-splitter connector → **left wing servo** + **right wing servo**
- **Mount one servo physically reversed** (rotate it 180° on its bracket, or reverse the horn
  attachment) so that a single signal moves the left aileron up while the right goes down and
  vice versa. This is the standard Y-cable setup — no code change needed, it's a mechanical fix.
- If the direction is wrong after mounting, swap which servo is reversed, or use the
  servo-reverse option on the FS-i6X for that channel.

### Full wiring table

| ESP32 pin | Connects to |
|---|---|
| GPIO 16 (RX2) | FS-iA10B iBus output (via 1 kΩ / 2 kΩ divider — iBus is 5 V) |
| GPIO 26 (RX1) | NEO-M8N GPS TX |
| GPIO 4  (TX1) | NEO-M8N GPS RX (optional) |
| GPIO 21 (SDA) | MPU-6050 SDA |
| GPIO 22 (SCL) | MPU-6050 SCL |
| GPIO 32 | Aileron Y-splitter signal |
| GPIO 25 | Elevator servo signal |
| GPIO 27 | ESC signal |
| USB / Serial | Telemetry radio (57 600 baud) for Mission Planner |

---

## FS-i6X transmitter channel assignment

Set on the transmitter itself (Functions → Aux. channels):

| Channel | Stick / Switch | FC function |
|---|---|---|
| CH1 | Right stick left/right | Aileron / roll |
| CH2 | Right stick up/down | Elevator / pitch |
| CH3 | Left stick up/down | Throttle |
| CH4 | Left stick left/right | Yaw (arm/disarm gesture only — no rudder) |
| CH5 | 3-position switch | MANUAL (down) / STABILIZE (mid) / AUTO (up) |
| CH6 | 2-position switch | LOITER (enable home loiter in RTH/AUTO) |

---

## Flight modes

| CH5 position | CH6 | Mode | Description |
|---|---|---|---|
| Down (-100) | — | MANUAL | Raw stick passthrough — for first glide tests |
| Mid (0) | — | STABILIZE | **Angle-mode self-levelling; user commands angles directly (up to ±30°); plane levels when stick released** |
| Up (+100) | — | AUTO | **Follows uploaded MAVLink waypoint mission; user inputs blended (30% user, 70% autopilot); can take manual control at any time** |
| Any | High | RTH + LOITER | GPS return-to-home + loiter around home; also auto-engaged on signal loss or GCS RTL command |

**Override priority (highest first):**
signal-loss failsafe → GCS RTL command → CH6 LOITER → pilot CH5 switch

If CH5 is flipped to AUTO but no mission is loaded, the FC falls back to RTH automatically.

---

## Arming

The motor is completely blocked until armed. Two methods:

**Stick gesture** (no telemetry radio required):
- Throttle at **minimum** + yaw **full right** held for 1 second → **ARMED**
- Throttle at **minimum** + yaw **full left** held for 1 second → **DISARMED**

**Mission Planner**: Actions tab → Arm / Disarm

> Signal loss during flight does NOT disarm — the plane needs throttle to fly home.

---

## ESC initialization (critical for generic ESCs)

**Your 30A ESC requires a boot signal or it will not arm.** The firmware now:
1. Initializes the ESC on first GPIO setup in `setup()` and sends **RC_MIN immediately**
2. Continues sending RC_MIN for ~500 ms during initialization
3. **Maintains a continuous ESC signal in every loop iteration** (RC_MIN when disarmed, user throttle when armed)

If the ESC still doesn't respond:
- Check wiring (GPIO 27 → ESC signal line)
- Verify 5V supply to the ESC
- Try a full power cycle (unplug battery, wait 5 s, reconnect)
- Some ESCs require a specific boot sequence; consult your ESC manual

---

## First power-up checklist

1. **Propeller off** for all bench work.
2. Power on with airframe flat and level — the IMU calibration runs for ~1.5 s on boot; keep it still.
3. Open Serial Monitor at 115 200 baud and confirm: `[IMU] Calibration done` then `[FC] Ready`.
4. **Confirm ESC beeps** — if the ESC is silent, check wiring and power supply.
5. In MANUAL mode, move sticks and confirm each control surface travels the right direction before switching to STABILIZE.
6. Go outdoors, wait for GPS 3D fix (cold start takes a few minutes). Home is auto-captured once HDOP < 3.0.

---

## STABILIZE mode — new behavior

**Old**: stick input was scaled then fed to PID (reduced authority)
**New**: stick input directly commands a **target angle**, hard-limited to ±30°.

- Full right stick → bank 30° right
- Mid stick → level flight (0° target)
- Release stick → plane auto-levels because target returns to 0°

This mimics quadcopter behavior: **the plane cannot stall or flip**, and autopilot takes over smoothly when you let go.

---

## AUTO mode — user control overlay

**Old**: autopilot had exclusive control
**New**: user inputs are **blended at 30% weight** with autopilot commands (70% autopilot).

- If autopilot is navigating and you grab the stick, the plane follows your input but also corrects course
- Release the stick and the plane returns to autopilot
- This allows you to take manual control in an emergency without disabling the flight plan

---

## Tunable parameters

All adjustable live from Mission Planner without reflashing. Saved to flash on every change.

| Param | Default | Meaning |
|---|---|---|
| `ROLL_KP` / `KI` / `KD` | 1.8 / 0.4 / 0.05 | Roll PID gains |
| `PITCH_KP` / `KI` / `KD` | 1.6 / 0.3 / 0.05 | Pitch PID gains |
| `STAB_ANGLE_MAX` | 30 ° | Full stick → this target angle |
| `WP_RADIUS_M` | 25 m | AUTO waypoint acceptance radius |
| `HOME_RADIUS_M` | 40 m | RTH loiter circle radius |
| `NAV_BANK_GAIN` | 0.6 | Heading-error → bank-command gain |
| `RTL_BANK_DEG` | 20 ° | Max bank during RTH |
| `AUTO_BANK_DEG` | 20 ° | Max bank during AUTO |
| `RTL_THR_US` | 1550 µs | Cruise throttle in RTH |
| `AUTO_THR_US` | 1600 µs | Cruise throttle in AUTO |
| `RC_TIMEOUT_MS` | 500 ms | Signal-loss failsafe trigger time |

**PID tuning order**: set KI = KD = 0, raise KP until it self-levels without oscillating,
add a little KD to damp overshoot, then a tiny KI only if there's a persistent steady lean.

---

## Phased test plan

1. **Glide test** — motor off, hand-launch, verify stable glide and correct surface response.
2. **MANUAL mode** — motor on (prop on!), confirm it flies safely on raw stick inputs.
3. **STABILIZE** — at safe altitude, verify self-levelling behaviour and angle limiting.
4. **RTH** — deliberately flip CH5 to RTH at altitude, confirm it turns toward home and loiters.
5. **LOITER** — flip CH6 to loiter and verify it orbits the home point.
6. **AUTO** — upload a tight 2-waypoint mission near the field, fly it with CH5 ready to override.

---

## Limitations

- **No altitude control** — there is no barometer in the parts list. Throttle is a fixed
  cruise value in RTH/AUTO. Waypoint altitude values are stored but not acted on. Adding a
  BMP280 and a throttle/pitch altitude-hold loop is the natural next step.
- **No rudder / yaw control** — turns are bank-only (matches the airframe design).
- **No geofencing, ADSB, or battery failsafe** — straightforward extensions for future work.
