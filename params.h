#pragma once
// Runtime-tunable configuration. Unlike config.h (pins, compile-time constants),
// everything here can be read and changed live from Mission Planner's parameter
// list (PARAM_REQUEST_LIST / PARAM_SET) and is persisted in ESP32 NVS flash via
// the Preferences library, so tuning survives a reboot without reflashing.

#include <Arduino.h>

struct Params {
  float rollKp, rollKi, rollKd;
  float pitchKp, pitchKi, pitchKd;
  float maxStabAngleDeg;
  float wpRadiusM;
  float homeRadiusM;
  float navBankGain;
  float rtlBankAngle;
  float autoBankAngle;
  float rtlCruiseThrottleUs;
  float autoCruiseThrottleUs;
  float rcTimeoutMs;
};

extern Params params;

void paramsLoadDefaults();
void paramsBegin();     // load from NVS, falling back to defaults for anything not yet saved
void paramsSave();      // persist current values to NVS

#define PARAM_COUNT 15

struct ParamDef {
  const char *name;  // MAVLink param_id, must be <=15 chars (NVS key limit) and unique
  float *value;
};

extern const ParamDef paramTable[PARAM_COUNT];

bool paramGetByIndex(uint16_t index, const char *&name, float &value);
bool paramGetByName(const char *name, float &value);
bool paramSetByName(const char *name, float value);
