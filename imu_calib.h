#pragma once
// =====================================================================
//  imu_calib.h — IMU calibration persistence module
//  Saves/loads accelerometer and gyroscope offsets to NVS flash
//  so they survive power cycles and plane installation moves.
// =====================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <MPU6050_light.h>

#define IMU_CALIB_NAMESPACE "imu_cal"

struct IMUOffsets {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
};

extern IMUOffsets imuOffsets;

// Load IMU offsets from NVS flash. Returns true if found, false if not yet calibrated.
bool imuCalibLoad(MPU6050 &mpu);

// Save IMU offsets to NVS flash from the MPU6050 object.
void imuCalibSave(MPU6050 &mpu);

// Clear saved calibration (useful for emergency reset).
void imuCalibClear();

// Check if calibration exists in flash.
bool imuCalibExists();
