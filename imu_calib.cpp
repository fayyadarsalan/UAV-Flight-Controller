#include "imu_calib.h"

IMUOffsets imuOffsets;
static Preferences imuPrefs;

bool imuCalibLoad(MPU6050 &mpu) {
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
  
  // Apply offsets to the MPU6050 object
  mpu.setAccOffsets(imuOffsets.accelX, imuOffsets.accelY, imuOffsets.accelZ);
  mpu.setGyroOffsets(imuOffsets.gyroX, imuOffsets.gyroY, imuOffsets.gyroZ);
  
  imuPrefs.end();
  return true;
}

void imuCalibSave(MPU6050 &mpu) {
  // Read current offsets from the MPU6050 object
  // MPU6050_light stores offsets internally, we need to save them
  // The library stores them as getAccXoffset(), getAccYoffset(), getAccZoffset()
  // and getGyroXoffset(), getGyroYoffset(), getGyroZoffset()
  
  imuPrefs.begin(IMU_CALIB_NAMESPACE, false);  // read-write
  
  // Get the offsets that were just calculated by calcOffsets()
  imuOffsets.accelX = mpu.getAccXoffset();
  imuOffsets.accelY = mpu.getAccYoffset();
  imuOffsets.accelZ = mpu.getAccZoffset();
  imuOffsets.gyroX  = mpu.getGyroXoffset();
  imuOffsets.gyroY  = mpu.getGyroYoffset();
  imuOffsets.gyroZ  = mpu.getGyroZoffset();
  
  imuPrefs.putFloat("accelX", imuOffsets.accelX);
  imuPrefs.putFloat("accelY", imuOffsets.accelY);
  imuPrefs.putFloat("accelZ", imuOffsets.accelZ);
  imuPrefs.putFloat("gyroX", imuOffsets.gyroX);
  imuPrefs.putFloat("gyroY", imuOffsets.gyroY);
  imuPrefs.putFloat("gyroZ", imuOffsets.gyroZ);
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
