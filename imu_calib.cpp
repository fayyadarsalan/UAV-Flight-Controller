#include "imu_calib.h"

IMUOffsets imuOffsets;
static Preferences imuPrefs;

bool imuCalibLoad() {
  imuPrefs.begin(IMU_CALIB_NAMESPACE, true);  // read-only
  
  // Check if calibration exists by looking for a marker key
  if (!imuPrefs.isKey("valid")) {
    imuPrefs.end();
    return false;
  }
  
  imuOffsets.accelX = imuPrefs.getFloat("accelX", 0.0f);
  imuOffsets.accelY = imuPrefs.getFloat("accelY", 0.0f);
  imuOffsets.accelZ = imuPrefs.getFloat("accelZ", 0.0f);
  imuOffsets.gyroX  = imuPrefs.getFloat("gyroX", 0.0f);
  imuOffsets.gyroY  = imuPrefs.getFloat("gyroY", 0.0f);
  imuOffsets.gyroZ  = imuPrefs.getFloat("gyroZ", 0.0f);
  
  imuPrefs.end();
  return true;
}

void imuCalibSave(float accelX, float accelY, float accelZ,
                  float gyroX, float gyroY, float gyroZ) {
  imuPrefs.begin(IMU_CALIB_NAMESPACE, false);  // read-write
  
  imuPrefs.putFloat("accelX", accelX);
  imuPrefs.putFloat("accelY", accelY);
  imuPrefs.putFloat("accelZ", accelZ);
  imuPrefs.putFloat("gyroX", gyroX);
  imuPrefs.putFloat("gyroY", gyroY);
  imuPrefs.putFloat("gyroZ", gyroZ);
  imuPrefs.putInt("valid", 1);  // marker to indicate calibration exists
  
  imuPrefs.end();
}

void imuCalibClear() {
  imuPrefs.begin(IMU_CALIB_NAMESPACE, false);
  imuPrefs.clear();
  imuPrefs.end();
}

bool imuCalibExists() {
  imuPrefs.begin(IMU_CALIB_NAMESPACE, true);
  bool exists = imuPrefs.isKey("valid");
  imuPrefs.end();
  return exists;
}
