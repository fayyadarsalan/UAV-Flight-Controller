#pragma once
#include <Arduino.h>

#define MAX_WAYPOINTS 30

struct Waypoint {
  int32_t lat_e7;
  int32_t lon_e7;
  float altM;   // meters — stored from the mission item but not used for altitude control yet;
                // this airframe has no altitude sensor/throttle-based altitude hold. Reserved
                // for a future barometer/altitude-hold addition.
};

class WaypointMission {
public:
  void clear();

  // Called when MISSION_COUNT arrives from the GCS: we're about to receive `count` items.
  void beginUpload(uint16_t count);
  bool isUploading() const { return _uploading; }
  uint16_t expectedCount() const { return _expectedCount; }

  bool setItem(uint16_t seq, int32_t lat_e7, int32_t lon_e7, float altM);
  void endUpload() { _uploading = false; }

  bool getItem(uint16_t seq, int32_t &lat_e7, int32_t &lon_e7, float &altM) const;
  uint16_t count() const { return _count; }

  uint16_t currentSeq() const { return _currentSeq; }
  void setCurrentSeq(uint16_t seq);

  // Moves to the next waypoint. Returns true if it advanced to a new one,
  // false if already at (and staying at) the last waypoint.
  bool advance();
  bool isComplete() const { return _complete; }
  void restart();   // rewind to seq 0 and clear the complete flag

private:
  Waypoint _items[MAX_WAYPOINTS];
  uint16_t _count = 0;
  uint16_t _expectedCount = 0;
  bool _uploading = false;
  uint16_t _currentSeq = 0;
  bool _complete = false;
};

extern WaypointMission mission;
