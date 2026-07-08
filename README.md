# UAV Flight Controller — ESP32-S3 (Full MAVLink Edition)

## Quick summary of changes in this version

| What changed | Old | New |
|---|---|---|
| Wing servos | 2 independent outputs | 1 output → Y-splitter → both servos |
| MAVLink layer | Hand-built protocol | Official MAVLink Arduino library |
| Waypoint upload | Broken (bad CRC_EXTRA by hand) | Fixed — uses official library handshake |

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
| CH5 | 3-position switch | MANUAL → STABILIZE → RTH |
| CH6 | 2-position switch | AUTO (waypoint mission) when high |

---

## Flight modes

| CH5 position | CH6 | Mode | Description |
|---|---|---|---|
| Down | — | MANUAL | Raw stick passthrough — for first glide tests |
| Centre | — | STABILIZE | Angle-mode self-levelling assist (PID) |
| Up | — | RTH | GPS return-to-home + loiter |
| Any | High | AUTO | Follows uploaded MAVLink waypoint mission |

**Override priority (highest first):**
signal-loss failsafe → GCS RTL command → CH6 AUTO → CH5 switch

If CH6 is flipped to AUTO but no mission is loaded, the FC falls back to RTH automatically.

---

## Arming

The motor is completely blocked until armed. Two methods:

**Stick gesture** (no telemetry radio required):
- Throttle at **minimum** + yaw **full right** held for 1 second → **ARMED**
- Throttle at **minimum** + yaw **full left** held for 1 second → **DISARMED**

**Mission Planner**: Actions tab → Arm / Disarm

> Signal loss during flight does NOT disarm — the plane needs throttle to fly home.

---

## First power-up checklist

1. **Propeller off** for all bench work.
2. Power on with airframe flat and level — the IMU calibration runs for ~1.5 s on boot; keep it still.
3. Open Serial Monitor at 115 200 baud and confirm: `[IMU] Calibration done` then `[FC] Ready`.
4. In MANUAL mode, move sticks and confirm each control surface travels the right direction before switching to STABILIZE.
5. Go outdoors, wait for GPS 3D fix (cold start takes a few minutes). Home is auto-captured once HDOP < 3.0.

---

## Mission Planner workflow

1. Connect: **serial port your radio appears on**, baud **57 600**.  
   You should see the vehicle heartbeat and the HUD update with roll/pitch.

2. **Parameters**: Config tab → Full Parameter List.  
   All 15 tunable parameters appear. Changes save to flash immediately.

3. **Mission upload**:
   - Flight Plan tab → place waypoints on the map
   - Click **Write WPs** — Mission Planner sends `MISSION_COUNT` then `MISSION_ITEM_INT` messages for each waypoint. The FC acknowledges each one and responds with `MISSION_ACK`.
   - If upload stalls, try clicking Write WPs again — one retry is usually enough.

4. **Fly the mission**: flip CH6 high (or Actions → Start Mission).  
   The FC reports `MISSION_ITEM_REACHED` as each waypoint is reached; the current-WP marker moves in Mission Planner.

5. **Read WPs**: pulls the stored mission back from the FC to verify it.

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
3. **STABILIZE** — at safe altitude, verify self-levelling behaviour.
4. **RTH** — deliberately flip CH5 to RTH at altitude, confirm it turns toward home and loiters.
5. **AUTO** — upload a tight 2-waypoint mission near the field, fly it with CH5 ready to override.

---

## Limitations

- **No altitude control** — there is no barometer in the parts list. Throttle is a fixed
  cruise value in RTH/AUTO. Waypoint altitude values are stored but not acted on. Adding a
  BMP280 and a throttle/pitch altitude-hold loop is the natural next step.
- **No rudder / yaw** — turns are bank-only (matches the airframe design).
- **No geofencing, ADSB, or battery failsafe** — straightforward extensions for future work.
