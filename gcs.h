#pragma once
// =====================================================================
//  gcs.h — Ground Control Station interface using the official MAVLink
//  Arduino library (install "MAVLink" from the Arduino Library Manager).
//
//  Handles the full MAVLink protocol session with Mission Planner /
//  QGroundControl:
//    • Telemetry:  HEARTBEAT, ATTITUDE, GLOBAL_POSITION_INT, GPS_RAW_INT,
//                  SYS_STATUS, MISSION_CURRENT, MISSION_ITEM_REACHED
//    • Missions:   upload (MISSION_COUNT → MISSION_REQUEST_INT → MISSION_ITEM_INT),
//                  download, MISSION_CLEAR_ALL, MISSION_SET_CURRENT
//    • Parameters: PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET
//                  (all 15 tunable params, persisted to NVS flash)
//    • Commands:   COMMAND_LONG — ARM/DISARM, RTL, MISSION_START
// =====================================================================

#include <Arduino.h>

namespace GCS {
  // Call once in setup() before using any other function.
  void begin(Stream &port, uint8_t sysId, uint8_t compId);

  // Non-blocking; parses all available bytes and dispatches messages.
  // Call every loop iteration (before the rate-limited control section).
  void update();

  // ── Outgoing telemetry ──────────────────────────────────────────────
  void sendHeartbeat(uint8_t customMode, bool armed);

  void sendAttitude(float rollRad, float pitchRad, float yawRad,
                     float rollSpeedRadS, float pitchSpeedRadS, float yawSpeedRadS);

  void sendGlobalPositionInt(int32_t lat_e7, int32_t lon_e7,
                              int32_t altMm, int32_t relAltMm,
                              int16_t vxCms, int16_t vyCms, int16_t vzCms,
                              uint16_t hdgCdeg);

  void sendGpsRawInt(int32_t lat_e7, int32_t lon_e7, int32_t altMm,
                      uint8_t fixType, uint8_t satellites);

  void sendSysStatus();

  void sendMissionCurrent(uint16_t seq);

  void sendMissionItemReached(uint16_t seq);

  // ── Arm / disarm state (settable from stick gesture OR MAVLink) ────
  bool armed();
  void setArmed(bool state);

  // ── Latched mode-override flags set by incoming COMMAND_LONG ───────
  // cleared by the main sketch once it has acted on them.
  bool rtlCommanded();
  void clearRtlCommand();

  bool missionStartCommanded();
  void clearMissionStartCommand();
}
