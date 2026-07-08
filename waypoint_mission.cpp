#include "waypoint_mission.h"

WaypointMission mission;

void WaypointMission::clear() {
  _count = 0;
  _expectedCount = 0;
  _uploading = false;
  _currentSeq = 0;
  _complete = false;
}

void WaypointMission::beginUpload(uint16_t count) {
  _expectedCount = (count > MAX_WAYPOINTS) ? MAX_WAYPOINTS : count;
  _count = 0;
  _uploading = true;
  _currentSeq = 0;
  _complete = false;
}

bool WaypointMission::setItem(uint16_t seq, int32_t lat_e7, int32_t lon_e7, float altM) {
  if (seq >= MAX_WAYPOINTS) return false;
  _items[seq].lat_e7 = lat_e7;
  _items[seq].lon_e7 = lon_e7;
  _items[seq].altM = altM;
  if (seq + 1 > _count) _count = seq + 1;
  return true;
}

bool WaypointMission::getItem(uint16_t seq, int32_t &lat_e7, int32_t &lon_e7, float &altM) const {
  if (seq >= _count) return false;
  lat_e7 = _items[seq].lat_e7;
  lon_e7 = _items[seq].lon_e7;
  altM = _items[seq].altM;
  return true;
}

void WaypointMission::setCurrentSeq(uint16_t seq) {
  if (seq < _count) {
    _currentSeq = seq;
    _complete = false;
  }
}

bool WaypointMission::advance() {
  if (_currentSeq + 1 < _count) {
    _currentSeq++;
    return true;
  }
  _complete = true;
  return false;
}

void WaypointMission::restart() {
  _currentSeq = 0;
  _complete = false;
}
