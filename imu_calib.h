#pragma once
// IMU calibration persistence module
// Saves/loads accelerometer and gyroscope offsets to NVS flash
// so they survive power cycles and plane installation moves

#include <Arduino.h>
#include <Preferences.h>

#define IMU_CALIB_NAMESPACE "imu_cal"

struct IMUOffsets {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
};

extern IMUOffsets imuOffsets;

// Load IMU offsets from NVS flash. Returns true if found, false if not yet calibrated.
bool imuCalibLoad();

// Save IMU offsets to NVS flash.
void imuCalibSave(float accelX, float accelY, float accelZ,
                  float gyroX, float gyroY, float gyroZ);

// Clear saved calibration (useful for emergency reset).
void imuCalibClear();

// Check if calibration exists in flash.
bool imuCalibExists();
