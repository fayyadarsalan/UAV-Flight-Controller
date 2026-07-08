// =====================================================================
//  gcs.cpp — MAVLink session using the official Arduino MAVLink library.
//
//  ⚠️  LIBRARY INSTALLATION REQUIRED ⚠️
//    1. Open Arduino IDE
//    2. Go to: Sketch → Include Library → Manage Libraries
//    3. Search for "MAVLink"
//    4. Install "MAVLink" by Lorenz Meier (version 2.0 or later)
//    5. If you see "common/mavlink.h" in the library folder, use that path instead
//
//  If still getting "file not found" after install:
//    - Try: #include <common/mavlink.h>
//    - Or check Arduino preferences for the Library path
//
//  WHY WAYPOINTS NOW WORK:
//    Official library handles all CRC, framing, and field layouts correctly.
// =====================================================================

// ⭐ CRITICAL: mavlink.h MUST come FIRST, before everything else
#include <MAVLink.h>

// Now include project headers
#include "gcs.h"
#include "waypoint_mission.h"
#include "params.h"
#include <string.h>

// ── module-level state (anonymous namespace keeps it file-scoped) ──
namespace {

  Stream   *_port    = nullptr;
  uint8_t   _sysId  = 1;
  uint8_t   _compId = 1;

  bool _armed               = false;
  bool _rtlCommanded        = false;
  bool _missionStartCmd     = false;

  // Remembered GCS address for targeting replies during an upload.
  uint8_t _uploadGcsSys  = 255;
  uint8_t _uploadGcsComp = 0;

  // ── helpers ────────────────────────────────────────────────────────

  void transmit(mavlink_message_t &msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (_port) _port->write(buf, len);
  }

  // Request the next waypoint from the GCS (upload flow).
  void requestWaypointItem(uint16_t seq) {
    mavlink_message_t msg;
    mavlink_msg_mission_request_int_pack(
        _sysId, _compId, &msg,
        _uploadGcsSys, _uploadGcsComp,
        seq,
        MAV_MISSION_TYPE_MISSION);
    transmit(msg);
  }

  // Acknowledge a completed upload or a clear-all.
  void sendMissionAck(uint8_t targetSys, uint8_t targetComp, uint8_t result) {
    mavlink_message_t msg;
    mavlink_msg_mission_ack_pack(
        _sysId, _compId, &msg,
        targetSys, targetComp,
        result,
        MAV_MISSION_TYPE_MISSION,
        0);  // mission_request_only (0 = not a request, it's a reply)
    transmit(msg);
  }

  // Build and send a PARAM_VALUE for entry [index] in the param table.
  void sendParamValue(uint16_t index) {
    const char *name;
    float value;
    if (!paramGetByIndex(index, name, value)) return;
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(
        _sysId, _compId, &msg,
        name, value,
        MAV_PARAM_TYPE_REAL32,
        PARAM_COUNT,
        index);
    transmit(msg);
  }

  // Send one MISSION_ITEM_INT from the stored mission to targetSys/Comp.
  void sendStoredMissionItem(uint8_t targetSys, uint8_t targetComp, uint16_t seq) {
    int32_t lat_e7, lon_e7;
    float altM;
    if (!mission.getItem(seq, lat_e7, lon_e7, altM)) return;

    mavlink_message_t msg;
    mavlink_msg_mission_item_int_pack(
        _sysId, _compId, &msg,
        targetSys, targetComp,
        seq,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_NAV_WAYPOINT,
        0,              // current
        1,              // autocontinue
        0.0f, 0.0f, 0.0f, 0.0f,  // param1-4
        lat_e7, lon_e7, altM,
        MAV_MISSION_TYPE_MISSION);
    transmit(msg);
  }

  // ── upload-flow helper ─────────────────────────────────────────────
  // Called when a waypoint arrives (whether ITEM or ITEM_INT).
  void storeItemAndContinue(uint16_t seq, int32_t lat_e7, int32_t lon_e7, float altM) {
    mission.setItem(seq, lat_e7, lon_e7, altM);

    if (seq + 1 < mission.expectedCount()) {
      requestWaypointItem(seq + 1);
    } else {
      mission.endUpload();
      sendMissionAck(_uploadGcsSys, _uploadGcsComp, MAV_MISSION_ACCEPTED);
    }
  }

  // ── incoming message handlers ──────────────────────────────────────

  // GCS is about to upload `count` waypoints — start the handshake.
  void onMissionCount(const mavlink_message_t &inmsg) {
    mavlink_mission_count_t mc;
    mavlink_msg_mission_count_decode(&inmsg, &mc);

    _uploadGcsSys  = inmsg.sysid;
    _uploadGcsComp = inmsg.compid;

    if (mc.count == 0) {
      mission.clear();
      sendMissionAck(_uploadGcsSys, _uploadGcsComp, MAV_MISSION_ACCEPTED);
      return;
    }
    mission.beginUpload(mc.count);
    requestWaypointItem(0);   // ask for item 0 to start the chain
  }

  // Waypoint with integer lat/lon (modern Mission Planner default).
  void onMissionItemInt(const mavlink_message_t &inmsg) {
    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(&inmsg, &item);
    if (!mission.isUploading()) return;
    storeItemAndContinue(item.seq, item.x, item.y, item.z);
  }

  // Waypoint with float lat/lon (older GCS versions / backward compat).
  void onMissionItem(const mavlink_message_t &inmsg) {
    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(&inmsg, &item);
    if (!mission.isUploading()) return;
    int32_t lat_e7 = (int32_t)(item.x * 1.0e7f);
    int32_t lon_e7 = (int32_t)(item.y * 1.0e7f);
    storeItemAndContinue(item.seq, lat_e7, lon_e7, item.z);
  }

  // GCS wants to download the stored mission — reply with item count.
  void onMissionRequestList(const mavlink_message_t &inmsg) {
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(
        _sysId, _compId, &msg,
        inmsg.sysid, inmsg.compid,
        mission.count(),
        MAV_MISSION_TYPE_MISSION,
        0);  // mission_request_only
    transmit(msg);
  }

  // GCS is asking for a specific item (int version — used by newer GCS).
  void onMissionRequestInt(const mavlink_message_t &inmsg) {
    mavlink_mission_request_int_t req;
    mavlink_msg_mission_request_int_decode(&inmsg, &req);
    sendStoredMissionItem(inmsg.sysid, inmsg.compid, req.seq);
  }

  // GCS is asking for a specific item (float version — backward compat).
  void onMissionRequest(const mavlink_message_t &inmsg) {
    mavlink_mission_request_t req;
    mavlink_msg_mission_request_decode(&inmsg, &req);
    sendStoredMissionItem(inmsg.sysid, inmsg.compid, req.seq);
  }

  void onMissionClearAll(const mavlink_message_t &inmsg) {
    mission.clear();
    sendMissionAck(inmsg.sysid, inmsg.compid, MAV_MISSION_ACCEPTED);
  }

  void onMissionSetCurrent(const mavlink_message_t &inmsg) {
    mavlink_mission_set_current_t sc;
    mavlink_msg_mission_set_current_decode(&inmsg, &sc);
    mission.setCurrentSeq(sc.seq);

    mavlink_message_t msg;
    mavlink_msg_mission_current_pack(
        _sysId, _compId, &msg,
        sc.seq,                 // seq (current waypoint number)
        mission.count(),        // total (total number of waypoints)
        MAV_MISSION_TYPE_MISSION, // mission_type
        0,                      // fence_type (unused, 0)
        0, 0, 0);               // extended_param1/2/3 (unused)
    transmit(msg);
  }

  void onParamRequestList(const mavlink_message_t & /*inmsg*/) {
    for (uint16_t i = 0; i < PARAM_COUNT; i++) sendParamValue(i);
  }

  void onParamRequestRead(const mavlink_message_t &inmsg) {
    mavlink_param_request_read_t req;
    mavlink_msg_param_request_read_decode(&inmsg, &req);

    if (req.param_index >= 0) {
      sendParamValue((uint16_t)req.param_index);
    } else {
      // Look up by name string.
      char name[17] = {};
      strncpy(name, req.param_id, 16);
      for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        const char *n; float v;
        if (paramGetByIndex(i, n, v) && strncmp(n, name, 16) == 0) {
          sendParamValue(i);
          break;
        }
      }
    }
  }

  void onParamSet(const mavlink_message_t &inmsg) {
    mavlink_param_set_t ps;
    mavlink_msg_param_set_decode(&inmsg, &ps);

    char name[17] = {};
    strncpy(name, ps.param_id, 16);

    if (paramSetByName(name, ps.param_value)) {
      paramsSave();
      // Echo the new value back so Mission Planner confirms the update.
      for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        const char *n; float v;
        if (paramGetByIndex(i, n, v) && strncmp(n, name, 16) == 0) {
          sendParamValue(i);
          break;
        }
      }
    }
  }

  void onCommandLong(const mavlink_message_t &inmsg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&inmsg, &cmd);

    uint8_t result = MAV_RESULT_ACCEPTED;

    switch (cmd.command) {
      case MAV_CMD_COMPONENT_ARM_DISARM:
        _armed = (cmd.param1 > 0.5f);
        break;
      case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        _rtlCommanded = true;
        break;
      case MAV_CMD_MISSION_START:
        if (mission.count() > 0) {
          _missionStartCmd = true;
          mission.restart();
        } else {
          result = MAV_RESULT_FAILED;
        }
        break;
      default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }

    // Reply with COMMAND_ACK so Mission Planner doesn't retry.
    mavlink_message_t ack;
    mavlink_msg_command_ack_pack(
        _sysId, _compId, &ack,
        cmd.command,
        result,
        255,            // progress: 255 = unknown
        0,              // result_param2
        inmsg.sysid,
        inmsg.compid);
    transmit(ack);
  }

  // ── central dispatcher ─────────────────────────────────────────────
  void dispatch(mavlink_message_t &msg) {
    switch (msg.msgid) {
      // Mission upload (GCS → FC)
      case MAVLINK_MSG_ID_MISSION_COUNT:        onMissionCount(msg);       break;
      case MAVLINK_MSG_ID_MISSION_ITEM_INT:     onMissionItemInt(msg);     break;
      case MAVLINK_MSG_ID_MISSION_ITEM:         onMissionItem(msg);        break;
      // Mission download (FC → GCS)
      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: onMissionRequestList(msg); break;
      case MAVLINK_MSG_ID_MISSION_REQUEST_INT:  onMissionRequestInt(msg);  break;
      case MAVLINK_MSG_ID_MISSION_REQUEST:      onMissionRequest(msg);     break;
      // Mission management
      case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:    onMissionClearAll(msg);    break;
      case MAVLINK_MSG_ID_MISSION_SET_CURRENT:  onMissionSetCurrent(msg);  break;
      // Parameter protocol
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:   onParamRequestList(msg);   break;
      case MAVLINK_MSG_ID_PARAM_REQUEST_READ:   onParamRequestRead(msg);   break;
      case MAVLINK_MSG_ID_PARAM_SET:            onParamSet(msg);           break;
      // Command protocol
      case MAVLINK_MSG_ID_COMMAND_LONG:         onCommandLong(msg);        break;
      default: break;
    }
  }

} // anonymous namespace

// =====================================================================
//  Public API
// =====================================================================

void GCS::begin(Stream &port, uint8_t sysId, uint8_t compId) {
  _port   = &port;
  _sysId  = sysId;
  _compId = compId;
}

void GCS::update() {
  if (!_port) return;
  mavlink_message_t msg;
  mavlink_status_t  status;
  while (_port->available()) {
    uint8_t c = (uint8_t)_port->read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      dispatch(msg);
    }
  }
}

// ── Telemetry senders ──────────────────────────────────────────────────

void GCS::sendHeartbeat(uint8_t customMode, bool armed) {
  mavlink_message_t msg;
  uint8_t baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                   | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
  mavlink_msg_heartbeat_pack(
      _sysId, _compId, &msg,
      MAV_TYPE_FIXED_WING,
      MAV_AUTOPILOT_GENERIC,
      baseMode,
      (uint32_t)customMode,
      armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
  transmit(msg);
}

void GCS::sendAttitude(float rollRad, float pitchRad, float yawRad,
                        float rollSpeedRadS, float pitchSpeedRadS, float yawSpeedRadS) {
  mavlink_message_t msg;
  mavlink_msg_attitude_pack(
      _sysId, _compId, &msg,
      millis(),
      rollRad, pitchRad, yawRad,
      rollSpeedRadS, pitchSpeedRadS, yawSpeedRadS);
  transmit(msg);
}

void GCS::sendGlobalPositionInt(int32_t lat_e7, int32_t lon_e7,
                                  int32_t altMm, int32_t relAltMm,
                                  int16_t vxCms, int16_t vyCms, int16_t vzCms,
                                  uint16_t hdgCdeg) {
  mavlink_message_t msg;
  mavlink_msg_global_position_int_pack(
      _sysId, _compId, &msg,
      millis(),
      lat_e7, lon_e7, altMm, relAltMm,
      vxCms, vyCms, vzCms,
      hdgCdeg);
  transmit(msg);
}

void GCS::sendGpsRawInt(int32_t lat_e7, int32_t lon_e7, int32_t altMm,
                          uint8_t fixType, uint8_t satellites) {
  mavlink_message_t msg;
  mavlink_msg_gps_raw_int_pack(
      _sysId, _compId, &msg,
      0ULL,           // time_usec — not tracked
      fixType,
      lat_e7, lon_e7, altMm,
      UINT16_MAX,     // eph   — unknown
      UINT16_MAX,     // epv   — unknown
      UINT16_MAX,     // vel   — unknown
      UINT16_MAX,     // cog   — unknown
      satellites,
      0, 0, 0, 0, 0,  // alt_ellipsoid, h/v/vel_acc, hdg_acc
      0);             // yaw
  transmit(msg);
}

void GCS::sendSysStatus() {
  mavlink_message_t msg;
  mavlink_msg_sys_status_pack(
      _sysId, _compId, &msg,
      0, 0, 0,        // sensors present / enabled / health bitmasks
      0,              // load (0 = unknown)
      UINT16_MAX,     // voltage_battery — no sensor wired
      -1,             // current_battery — unknown
      -1,             // battery_remaining — unknown
      0, 0,           // drop_rate_comm, errors_comm
      0, 0, 0, 0,     // errors_count1-4
      0, 0, 0);       // extension fields
  transmit(msg);
}

void GCS::sendMissionCurrent(uint16_t seq) {
  mavlink_message_t msg;
  mavlink_msg_mission_current_pack(
      _sysId, _compId, &msg,
      seq,                      // seq (current waypoint number)
      mission.count(),          // total (total number of waypoints)
      MAV_MISSION_TYPE_MISSION, // mission_type
      0,                        // fence_type (unused, 0)
      0, 0, 0);                 // extended_param1/2/3 (unused)
  transmit(msg);
}

void GCS::sendMissionItemReached(uint16_t seq) {
  mavlink_message_t msg;
  mavlink_msg_mission_item_reached_pack(_sysId, _compId, &msg, seq);
  transmit(msg);
}

// ── State accessors ────────────────────────────────────────────────────

bool GCS::armed()                       { return _armed; }
void GCS::setArmed(bool state)          { _armed = state; }
bool GCS::rtlCommanded()                { return _rtlCommanded; }
void GCS::clearRtlCommand()             { _rtlCommanded = false; }
bool GCS::missionStartCommanded()       { return _missionStartCmd; }
void GCS::clearMissionStartCommand()    { _missionStartCmd = false; }
