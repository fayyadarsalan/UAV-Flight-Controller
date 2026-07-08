// =====================================================================
//  UAV_Flight_Controller.ino
//
//  Single-motor, fixed-wing pusher UAV on ESP32-S3.
//
//  Servos (3 total):
//    AILERON_PIN  → Y-splitter → left wing servo + right wing servo
//                  (mount one servo reversed so they deflect opposite
//                   directions from the same signal — standard Y-cable setup)
//    ELEVATOR_PIN → rear elevator servo
//    ESC_PIN      → brushless ESC
//
//  FS-i6X channel map:
//    CH1 left/right    → aileron / roll
//    CH2 up/down       → elevator / pitch
//    CH3               → throttle
//    CH4 yaw           → no rudder servo; used only for arm/disarm gesture
//    CH5 3-pos switch  → MANUAL / STABILIZE / AUTO (position-mapped)
//    CH6 2-pos switch  → LOITER (enable home loiter in RTH/AUTO)
//
//  Flight modes:
//    MANUAL      raw stick-to-servo passthrough (initial glide tests)
//    STABILIZE   angle-mode PID self-levelling with user commanded angles
//                (up to ±30° hard limit); plane levels when user releases stick
//    RTH         GPS return-to-home + loiter; also auto-engaged on
//                signal loss or a GCS RTL command
//    AUTO        follows uploaded MAVLink waypoint mission;
//                user inputs are blended with autopilot (70% autopilot, 30% user);
//                loiters at the last waypoint after completion
//
//  Arm / disarm gesture (works with no telemetry radio connected):
//    Throttle at min + yaw FULL RIGHT held 1 s  →  ARMED
//    Throttle at min + yaw FULL LEFT  held 1 s  →  DISARMED
//    Motor is hard-blocked whenever disarmed, in every mode.
//
//  IMU CALIBRATION (NEW):
//    Power on the board on your DESK (level), then:
//    1. Set CH5 to MANUAL (down)
//    2. Hold CH4 (yaw) FULL RIGHT for 3 seconds
//    3. Serial will print calibration progress
//    4. Once saved, remove power and install in plane at any angle
//    5. On next boot, saved calibration loads automatically
//
//  Required libraries (Arduino Library Manager):
//    • MAVLink        (official MAVLink library)
//    • MPU6050_light  (rfetick)
//    • TinyGPSPlus    (Mikal Hart)
//    • ESP32Servo     (madhephaestus)
// =====================================================================

#include <Wire.h>
#include <MPU6050_light.h>
#include <TinyGPSPlus.h>
#include <ESP32Servo.h>
#include <math.h>

#include "config.h"
#include "ibus.h"
#include "params.h"
#include "waypoint_mission.h"
#include "gcs.h"
#include "imu_calib.h"

// ── Hardware objects ──────────────────────────────────────────────────
MPU6050   mpu(Wire);
TinyGPSPlus gps;
IBusReader  ibus;

Servo servoAileron;   // drives both wing servos via Y-splitter
Servo servoElevator;
Servo esc;

// ── Flight state ──────────────────────────────────────────────────────
enum FlightMode : uint8_t {
  MODE_MANUAL    = 0,
  MODE_STABILIZE = 1,
  MODE_RTH       = 2,
  MODE_AUTO      = 3
};
FlightMode currentMode = MODE_STABILIZE;

double homeLat = 0.0, homeLon = 0.0;
bool   homeSet = false;

// ── PID running state ─────────────────────────────────────────────────
float rollIntegral   = 0.0f, rollLastError   = 0.0f;
float pitchIntegral  = 0.0f, pitchLastError  = 0.0f;

// ── Cached attitude (written every control cycle, read by telemetry) ──
float lastRollDeg = 0.0f, lastPitchDeg = 0.0f;

// ── Arm gesture timer ────────────────────────────────────────────────
unsigned long armGestureStartMs = 0;

// ── Loop timing ──────────────────────────────────────────────────────
unsigned long lastLoopMicros = 0;

// =====================================================================
//  Helpers
// =====================================================================

// Map a stick pulse (1000-2000 µs) to a target angle in degrees,
// limited by MAX_STAB_ANGLE_DEG (hard limit like a quadcopter).
float stickToAngle(uint16_t us) {
  float norm = ((float)constrain((int)us, RC_MIN, RC_MAX) - RC_MID)
               / (float)(RC_MAX - RC_MID);   // -1 … +1
  float targetAngle = norm * params.maxStabAngleDeg;
  // Hard limit: clamp to ±MAX_STAB_ANGLE_DEG
  return constrain(targetAngle, -MAX_STAB_ANGLE_DEG, MAX_STAB_ANGLE_DEG);
}

// Map a stick pulse directly to a servo angle (MANUAL passthrough).
int stickToServoDeg(uint16_t us) {
  return map(constrain((int)us, RC_MIN, RC_MAX), RC_MIN, RC_MAX,
             SERVO_CENTER_DEG - SERVO_THROW_DEG,
             SERVO_CENTER_DEG + SERVO_THROW_DEG);
}

// Decode the 3-position mode switch.
// Returns mode based on pulse width: -100 (down) = MANUAL, 0 (mid) = STABILIZE, +100 (up) = AUTO
FlightMode decodeModeSwitch(uint16_t us) {
  if (us < 1300) return MODE_MANUAL;      // low
  if (us < 1700) return MODE_STABILIZE;   // mid
  return MODE_AUTO;                        // high
}

// PID with clamped integrator.
float pidStep(float error, float &integral, float &lastErr,
              float kp, float ki, float kd, float dt) {
  integral += error * dt;
  integral  = constrain(integral, -20.0f, 20.0f);
  float d   = (dt > 0.0f) ? (error - lastErr) / dt : 0.0f;
  lastErr   = error;
  return kp * error + ki * integral + kd * d;
}

// Write throttle, forcing off when disarmed.
void writeThrottle(uint16_t us) {
  esc.writeMicroseconds(GCS::armed() ? us : RC_MIN);
}

// =====================================================================
//  IMU Calibration Menu — interactive setup
// =====================================================================
void showCalibMenu() {
  Serial.println("\n===============================================");
  Serial.println("         IMU CALIBRATION MENU");
  Serial.println("===============================================");
  Serial.println("Options:");
  Serial.println("  1 = Calibrate IMU now (keep level & still)");
  Serial.println("  2 = Load saved calibration");
  Serial.println("  3 = Clear saved calibration");
  Serial.println("  4 = Show current offsets");
  Serial.println("  5 = Continue with current settings");
  Serial.println("===============================================");
  Serial.println("Enter choice (1-5):");
}

void handleCalibMenu() {
  showCalibMenu();
  
  unsigned long timeout = millis() + 30000;  // 30 second timeout
  while (millis() < timeout) {
    if (Serial.available()) {
      char choice = Serial.read();
      Serial.println(choice);
      
      switch (choice) {
        case '1':
          Serial.println("\n[IMU] Performing fresh calibration...");
          Serial.println("Keep airframe LEVEL and STILL for 2 seconds...");
          delay(2000);
          mpu.calcOffsets(true, true);
          imuCalibSave(mpu);
          Serial.println("[IMU] ✓ Calibration saved to flash!");
          Serial.println("     You can now remove power and install in the plane.");
          break;
          
        case '2':
          if (imuCalibLoad(mpu)) {
            Serial.println("[IMU] ✓ Loading saved calibration...");
            Serial.println("[IMU] Offsets applied.");
          } else {
            Serial.println("[IMU] ✗ No saved calibration found.");
          }
          break;
          
        case '3':
          imuCalibClear();
          Serial.println("[IMU] ✓ Saved calibration cleared.");
          break;
          
        case '4':
          Serial.println("\nCurrent IMU Offsets:");
          Serial.print("  AccelX: "); Serial.println(mpu.getAccXoffset(), 6);
          Serial.print("  AccelY: "); Serial.println(mpu.getAccYoffset(), 6);
          Serial.print("  AccelZ: "); Serial.println(mpu.getAccZoffset(), 6);
          Serial.print("  GyroX:  "); Serial.println(mpu.getGyroXoffset(), 6);
          Serial.print("  GyroY:  "); Serial.println(mpu.getGyroYoffset(), 6);
          Serial.print("  GyroZ:  "); Serial.println(mpu.getGyroZoffset(), 6);
          Serial.println();
          showCalibMenu();
          continue;  // prompt again
          
        case '5':
          Serial.println("\n[IMU] Continuing with current offsets...");
          return;
          
        default:
          Serial.println("Invalid choice. Please enter 1-5.");
          showCalibMenu();
          continue;
      }
      
      // After choice, continue flight
      return;
    }
    delay(100);
  }
  
  Serial.println("\n[IMU] Menu timeout — continuing with current settings.");
}

// =====================================================================
//  Stabilization + surface mixing  (used by STABILIZE, RTH, AUTO)
// =====================================================================
void stabilizeAndFly(float targetRollDeg, float targetPitchDeg,
                      uint16_t throttleUs, float dt) {
  mpu.update();
  float roll  = mpu.getAngleX();
  float pitch = mpu.getAngleY();
  lastRollDeg  = roll;
  lastPitchDeg = pitch;

  float rollCorr = pidStep(targetRollDeg  - roll,
                            rollIntegral,  rollLastError,
                            params.rollKp, params.rollKi, params.rollKd, dt);
  float pitchCorr = pidStep(targetPitchDeg - pitch,
                              pitchIntegral, pitchLastError,
                              params.pitchKp, params.pitchKi, params.pitchKd, dt);

  rollCorr  = constrain(rollCorr,  -MAX_SERVO_TRIM_DEG, MAX_SERVO_TRIM_DEG);
  pitchCorr = constrain(pitchCorr, -MAX_SERVO_TRIM_DEG, MAX_SERVO_TRIM_DEG);

  // Single aileron output — Y-splitter distributes to both wing servos.
  // One servo is physically reversed so both wings deflect correctly.
  int aileronDeg  = (int)(SERVO_CENTER_DEG + rollCorr);
  int elevatorDeg = (int)(SERVO_CENTER_DEG + pitchCorr);

  servoAileron.write(constrain(aileronDeg,
                                SERVO_CENTER_DEG - SERVO_THROW_DEG,
                                SERVO_CENTER_DEG + SERVO_THROW_DEG));
  servoElevator.write(constrain(elevatorDeg,
                                  SERVO_CENTER_DEG - SERVO_THROW_DEG,
                                  SERVO_CENTER_DEG + SERVO_THROW_DEG));
  writeThrottle(throttleUs);
}

// =====================================================================
//  GPS navigation — shared by RTH and AUTO
//  Turns toward targetLat/Lon; once within radiusM, orbits at bankDeg.
// =====================================================================
void navigateTo(double targetLat, double targetLon,
                 float radiusM, float bankGain, float maxBankDeg,
                 uint16_t throttleUs, float dt, bool &reached) {
  reached = false;
  float targetRoll = 0.0f;

  if (gps.location.isValid()) {
    double curLat = gps.location.lat();
    double curLon = gps.location.lng();
    float  dist   = (float)TinyGPSPlus::distanceBetween(curLat, curLon, targetLat, targetLon);
    float  bear   = (float)TinyGPSPlus::courseTo(curLat, curLon, targetLat, targetLon);
    float  hdg    = gps.course.isValid() ? (float)gps.course.deg() : bear;
    float  hdgErr = fmod((bear - hdg + 540.0f), 360.0f) - 180.0f;  // signed -180…+180

    reached = (dist <= radiusM);
    targetRoll = reached
                 ? maxBankDeg                                              // orbit
                 : constrain(hdgErr * bankGain, -maxBankDeg, maxBankDeg); // turn
  }
  // No GPS fix yet → fly level and straight until one is acquired.
  stabilizeAndFly(targetRoll, 0.0f, throttleUs, dt);
}

// ── RTH ────────────────────────────────────────────────────────────────
void doRTH(float dt) {
  bool reached;
  navigateTo(
    homeSet ? homeLat : 0.0, homeSet ? homeLon : 0.0,
    params.homeRadiusM, params.navBankGain, params.rtlBankAngle,
    (uint16_t)params.rtlCruiseThrottleUs, dt, reached);
}

// ── AUTO: sequential waypoint following with user input blending ────
void doAuto(float dt, float userRollDeg, float userPitchDeg) {
  if (mission.count() == 0) { doRTH(dt); return; }  // no mission → failsafe

  int32_t lat_e7, lon_e7; float altM;
  if (!mission.getItem(mission.currentSeq(), lat_e7, lon_e7, altM)) {
    doRTH(dt); return;
  }

  bool reached;
  navigateTo(
    lat_e7 / 1.0e7, lon_e7 / 1.0e7,
    params.wpRadiusM, params.navBankGain, params.autoBankAngle,
    (uint16_t)params.autoCruiseThrottleUs, dt, reached);

  // Blend user input: 70% autopilot, 30% user commanded angles
  // (This happens after navigateTo() has already set the servo positions,
  //  so we read the current pitch/roll from the IMU and adjust)
  mpu.update();
  float currentRoll  = mpu.getAngleX();
  float currentPitch = mpu.getAngleY();
  float blendedRoll  = currentRoll  * (1.0f - USER_INPUT_BLEND) + userRollDeg * USER_INPUT_BLEND;
  float blendedPitch = currentPitch * (1.0f - USER_INPUT_BLEND) + userPitchDeg * USER_INPUT_BLEND;
  stabilizeAndFly(blendedRoll, blendedPitch, (uint16_t)params.autoCruiseThrottleUs, dt);

  if (reached) {
    // Fire each event once per waypoint.
    static uint16_t lastAnnounced = 0xFFFF;
    if (mission.currentSeq() != lastAnnounced) {
      lastAnnounced = mission.currentSeq();
      GCS::sendMissionItemReached(mission.currentSeq());
      if (mission.advance()) {
        GCS::sendMissionCurrent(mission.currentSeq());
      }
      // If advance() returned false we're at the last WP — loiter there.
    }
  }
}

// =====================================================================
//  GPS polling & home-capture
// =====================================================================
void updateGPS() {
  while (GPS_SERIAL.available()) gps.encode(GPS_SERIAL.read());

  if (!homeSet && gps.location.isValid()
      && gps.hdop.isValid() && gps.hdop.hdop() < 3.0) {
    homeLat = gps.location.lat();
    homeLon = gps.location.lng();
    homeSet = true;
    Serial.println("[GPS] Home captured.");
  }
}

// =====================================================================
//  Arm/disarm stick gesture
//  Throttle at min + yaw full-right for 1 s  →  arm
//  Throttle at min + yaw full-left  for 1 s  →  disarm
// =====================================================================
void checkArmGesture(uint16_t rawThrottle, uint16_t rawYaw) {
  bool tLow   = rawThrottle < (RC_MIN + 50);
  bool yRight = rawYaw      > (RC_MAX - 50);
  bool yLeft  = rawYaw      < (RC_MIN + 50);

  if (tLow && (yRight || yLeft)) {
    if (armGestureStartMs == 0) armGestureStartMs = millis();
    if (millis() - armGestureStartMs > 1000) {
      GCS::setArmed(yRight);   // right → arm, left → disarm
      armGestureStartMs = 0;
      Serial.println(yRight ? "[ARM] Armed via gesture."
                             : "[ARM] Disarmed via gesture.");
    }
  } else {
    armGestureStartMs = 0;
  }
}

// =====================================================================
//  setup()
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== UAV Flight Controller — booting ===");

  // Initialize ESC with a boot signal IMMEDIATELY
  // Many ESCs (like the 30A yellow ESC) require a signal at power-up to initialize
  pinMode(ESC_PIN, OUTPUT);
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(RC_MIN);  // Send minimum throttle signal at boot
  delay(100);  // Brief delay to ensure ESC receives the signal

  // Parameters (NVS flash — falls back to defaults on first boot).
  paramsBegin();

  // IMU
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  while (mpu.begin() != 0) {
    Serial.println("[IMU] MPU-6050 not found — check wiring. Retrying…");
    delay(500);
  }
  
  // Try to load saved calibration first
  if (imuCalibLoad(mpu)) {
    Serial.println("[IMU] ✓ Calibration loaded from flash.");
  } else {
    Serial.println("[IMU] No saved calibration found.");
  }

  // RC receiver
  ibus.begin(IBUS_SERIAL, IBUS_BAUD, IBUS_RX_PIN);

  // GPS
  GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Servos
  servoAileron.setPeriodHertz(50);
  servoAileron.attach(AILERON_PIN, 1000, 2000);
  servoElevator.setPeriodHertz(50);
  servoElevator.attach(ELEVATOR_PIN, 1000, 2000);

  // MAVLink / GCS
  GCS::begin(MAVLINK_SERIAL, MAVLINK_SYS_ID, MAVLINK_COMP_ID);

  // Continue sending ESC signal during initialization
  for (int i = 0; i < 5; i++) {
    esc.writeMicroseconds(RC_MIN);
    delay(100);
  }

  // ── Show calibration menu (interactive IMU setup) ──
  if (ENABLE_CALIB_MENU) {
    Serial.println("\n[SETUP] Waiting for RC input... (5 seconds)");
    delay(500);
    ibus.update();
    delay(500);
    ibus.update();
    delay(500);
    ibus.update();
    delay(500);
    ibus.update();
    delay(500);
    ibus.update();
    
    // Check if user triggered calibration: CH5 = MANUAL + CH4 (yaw) FULL RIGHT
    uint16_t rawModeSw = ibus.channel(CH_MODE_SWITCH);
    uint16_t rawYaw    = ibus.channel(CH_YAW);
    
    if (decodeModeSwitch(rawModeSw) == MODE_MANUAL && rawYaw > 1950) {
      Serial.println("[SETUP] Calibration trigger detected (CH5=MANUAL, CH4=FULL RIGHT).");
      handleCalibMenu();
    }
  }

  lastLoopMicros = micros();
  Serial.println("[FC] Ready — disarmed. Arm via stick gesture or Mission Planner.");
}

// =====================================================================
//  loop()
// =====================================================================
void loop() {
  // ── Fast parsers — run on every call, not rate-limited ───────────
  ibus.update();
  updateGPS();
  GCS::update();

  // ── Fixed-rate control loop ──────────────────────────────────────
  unsigned long now = micros();
  float dt = (float)(now - lastLoopMicros) * 1.0e-6f;
  if (dt < 1.0f / (float)LOOP_HZ) return;
  lastLoopMicros = now;

  // ── Read RC channels ─────────────────────────────────────────────
  uint16_t rawAileron  = ibus.channel(CH_AILERON);
  uint16_t rawElevator = ibus.channel(CH_ELEVATOR);
  uint16_t rawThrottle = ibus.channel(CH_THROTTLE);
  uint16_t rawYaw      = ibus.channel(CH_YAW);
  uint16_t rawModeSw   = ibus.channel(CH_MODE_SWITCH);
  uint16_t rawLoiterSw = ibus.channel(CH_LOITER_SWITCH);

  bool signalLost   = !ibus.isReceiving((uint32_t)params.rcTimeoutMs);
  bool loiterMode   = (rawLoiterSw > 1500);  // CH6 high = loiter enabled

  // Arm/disarm gesture only while we have a live signal.
  if (!signalLost) checkArmGesture(rawThrottle, rawYaw);

  // ── Mode arbitration ─────────────────────────────────────────────
  // Priority: signal-loss failsafe > GCS RTL > pilot switch > loiter override
  if (signalLost) {
    currentMode = MODE_RTH;
  } else if (GCS::rtlCommanded()) {
    currentMode = MODE_RTH;
  } else if (loiterMode) {
    // Loiter switch overrides to RTH (home loiter)
    currentMode = MODE_RTH;
  } else if (GCS::missionStartCommanded() && mission.count() > 0) {
    currentMode = MODE_AUTO;
  } else {
    currentMode = decodeModeSwitch(rawModeSw);
  }

  // Clear GCS overrides once the pilot's own switches have taken over.
  if (!signalLost) {
    if (decodeModeSwitch(rawModeSw) != MODE_RTH) GCS::clearRtlCommand();
    if (decodeModeSwitch(rawModeSw) == MODE_MANUAL) GCS::clearMissionStartCommand();
  }

  // ── User commanded angles (for blending in AUTO) ────────────────
  float userRollDeg  = stickToAngle(rawAileron);
  float userPitchDeg = stickToAngle(rawElevator);

  // ── Fly ──────────────────────────────────────────────────────────
  switch (currentMode) {

    case MODE_MANUAL:
      // Y-splitter: one output, one servo must be physically reversed.
      servoAileron.write(stickToServoDeg(rawAileron));
      servoElevator.write(stickToServoDeg(rawElevator));
      writeThrottle((uint16_t)constrain((int)rawThrottle, RC_MIN, RC_MAX));
      break;

    case MODE_STABILIZE:
      // User commands target angles (limited to ±30°).
      // Plane stabilizes at that angle and holds it even when stick is released.
      stabilizeAndFly(
        userRollDeg,
        userPitchDeg,
        (uint16_t)constrain((int)rawThrottle, RC_MIN, RC_MAX),
        dt);
      break;

    case MODE_RTH:
      doRTH(dt);
      break;

    case MODE_AUTO:
      // Autopilot navigates to waypoint, but user can override with stick inputs (blended)
      doAuto(dt, userRollDeg, userPitchDeg);
      break;
  }

  // Keep sending ESC signal in every loop to maintain connection
  if (!GCS::armed()) {
    esc.writeMicroseconds(RC_MIN);
  }

  // ── Scheduled MAVLink telemetry ───────────────────────────────────
  unsigned long ms = millis();

  // ATTITUDE at 10 Hz
  static unsigned long tAtt = 0;
  if (ms - tAtt >= 100) {
    tAtt = ms;
    GCS::sendAttitude(
      radians(lastRollDeg), radians(lastPitchDeg), 0.0f,
      0.0f, 0.0f, 0.0f);
  }

  // POSITION + GPS_RAW + SYS_STATUS at 2 Hz
  static unsigned long tPos = 0;
  if (ms - tPos >= 500) {
    tPos = ms;
    if (gps.location.isValid()) {
      int32_t lat_e7  = (int32_t)(gps.location.lat()  * 1.0e7);
      int32_t lon_e7  = (int32_t)(gps.location.lng()  * 1.0e7);
      int32_t altMm   = (int32_t)((gps.altitude.isValid()
                          ? gps.altitude.meters() : 0.0) * 1000.0);
      uint16_t hdgCdeg = (uint16_t)((gps.course.isValid()
                          ? gps.course.deg() : 0.0) * 100.0);
      uint8_t fixType = gps.location.isValid() ? 3 : 0;
      uint8_t sats    = gps.satellites.isValid()
                          ? (uint8_t)gps.satellites.value() : 0;

      GCS::sendGlobalPositionInt(lat_e7, lon_e7, altMm, altMm,
                                   0, 0, 0, hdgCdeg);
      GCS::sendGpsRawInt(lat_e7, lon_e7, altMm, fixType, sats);
    }
    GCS::sendSysStatus();
  }

  // HEARTBEAT at 1 Hz
  static unsigned long tHb = 0;
  if (ms - tHb >= 1000) {
    tHb = ms;
    GCS::sendHeartbeat((uint8_t)currentMode, GCS::armed());
  }
}
