#pragma once
// =====================================================================
//  config.h — pins, RC channel mapping, and system-level constants.
//  Runtime-tunable flight parameters (PID gains, radii, etc.) live in
//  params.h/.cpp instead, so they can be changed from Mission Planner
//  without reflashing. The DEFAULT_* values below only seed params.cpp
//  on first boot / after a param reset.
// =====================================================================

// ---------------- iBus receiver (FlySky FS-iA10B) ----------------
// iBus is 5V logic — use a voltage divider (e.g. 1k signal->RX, 2k RX->GND)
// before connecting to the ESP32 RX pin below.
#define IBUS_SERIAL       Serial2
#define IBUS_RX_PIN       18     // GPIO 16 (RX2) — iBus is 5V, use voltage divider
#define IBUS_BAUD         115200

// ---------------- GPS (u-blox NEO-M8N) ----------------
#define GPS_SERIAL        Serial1
#define GPS_RX_PIN        16     // GPIO 26 (RX1) — GPS TX to ESP32 RX
#define GPS_TX_PIN        17     // GPIO 4  (TX1) — ESP32 TX to GPS RX (optional, GPS is read-only here)
#define GPS_BAUD          9600

// ---------------- MAVLink link ----------------
// Wire a telemetry radio (or ESP32 USB/Wi-Fi bridge) here for Mission Planner / QGC.
#define MAVLINK_SERIAL    Serial
#define MAVLINK_BAUD      57600
#define MAVLINK_SYS_ID    1
#define MAVLINK_COMP_ID   1

// ---------------- I2C (MPU-6050) ----------------
#define I2C_SDA_PIN       9     // GPIO 21 — I2C SDA
#define I2C_SCL_PIN       8     // GPIO 22 — I2C SCL

// ---- Servo / ESC outputs — 2 servos via Y-splitter + 1 elevator + ESC ----
// AILERON_PIN drives a Y-splitter; one servo is physically reversed.
#define AILERON_PIN       4     // GPIO 32 — Y-splitter → left wing + right wing servos
#define ELEVATOR_PIN      5     // GPIO 25 — rear elevator servo
#define ESC_PIN           6     // GPIO 27 — brushless ESC

// ---------------- RC channel indices (iBus, 0-based) — FS-i6X mapping ----------------
#define CH_AILERON         0     // CH1: left-right
#define CH_ELEVATOR        1     // CH2: up-down
#define CH_THROTTLE        2     // CH3: throttle
#define CH_YAW             3     // CH4: yaw — no rudder servo on this airframe; read but only
                                 //       used for the arm/disarm stick gesture, not control
#define CH_MODE_SWITCH     4     // CH5: 3-position switch — MANUAL(-100) / STABILIZE(0) / AUTO(+100)
#define CH_LOITER_SWITCH   5     // CH6: 2-position switch — loiter mode enable

// ---- RC pulse range (microseconds) ----
#define RC_MIN              1000
#define RC_MID              1500
#define RC_MAX              2000

#define SERVO_CENTER_DEG     90
#define SERVO_THROW_DEG      45  // ±45 deg from center = 90 deg total throw
#define LOOP_HZ              200

// ---- Stabilization limits ----
#define MAX_STAB_ANGLE_DEG   30  // Hard limit on commanded angle in STABILIZE mode
#define USER_INPUT_BLEND     0.7f  // In AUTO mode, blend user input at this weight (0.0-1.0)

// ---- Default parameter values (seeded into NVS on first boot) ----
#define DEFAULT_ROLL_KP           1.8f
#define DEFAULT_ROLL_KI           0.4f
#define DEFAULT_ROLL_KD           0.05f
#define DEFAULT_PITCH_KP          1.6f
#define DEFAULT_PITCH_KI          0.3f
#define DEFAULT_PITCH_KD          0.05f
#define DEFAULT_STAB_ANGLE_MAX    30.0f     // deg, full stick -> this target angle
#define DEFAULT_WP_RADIUS_M       25.0f     // waypoint "arrived" radius
#define DEFAULT_HOME_RADIUS_M     40.0f     // RTH loiter radius
#define DEFAULT_NAV_BANK_GAIN     0.6f      // heading-error(deg) -> bank-command(deg) gain
#define DEFAULT_RTL_BANK_DEG      20.0f     // max bank while navigating/loitering home
#define DEFAULT_AUTO_BANK_DEG     20.0f     // max bank while navigating/loitering a waypoint
#define DEFAULT_RTL_THR_US        1550.0f
#define DEFAULT_AUTO_THR_US       1600.0f
#define DEFAULT_RC_TIMEOUT_MS     500.0f    // no iBus frame for this long -> failsafe RTH

#define MAX_SERVO_TRIM_DEG        25.0f     // PID authority around servo center
