#include "params.h"
#include "config.h"
#include <Preferences.h>
#include <string.h>

Params params;
static Preferences prefs;

const ParamDef paramTable[PARAM_COUNT] = {
  {"ROLL_KP",        &params.rollKp},
  {"ROLL_KI",        &params.rollKi},
  {"ROLL_KD",        &params.rollKd},
  {"PITCH_KP",       &params.pitchKp},
  {"PITCH_KI",       &params.pitchKi},
  {"PITCH_KD",       &params.pitchKd},
  {"STAB_ANGLE_MAX", &params.maxStabAngleDeg},
  {"WP_RADIUS_M",    &params.wpRadiusM},
  {"HOME_RADIUS_M",  &params.homeRadiusM},
  {"NAV_BANK_GAIN",  &params.navBankGain},
  {"RTL_BANK_DEG",   &params.rtlBankAngle},
  {"AUTO_BANK_DEG",  &params.autoBankAngle},
  {"RTL_THR_US",     &params.rtlCruiseThrottleUs},
  {"AUTO_THR_US",    &params.autoCruiseThrottleUs},
  {"RC_TIMEOUT_MS",  &params.rcTimeoutMs},
};

void paramsLoadDefaults() {
  params.rollKp = DEFAULT_ROLL_KP;
  params.rollKi = DEFAULT_ROLL_KI;
  params.rollKd = DEFAULT_ROLL_KD;
  params.pitchKp = DEFAULT_PITCH_KP;
  params.pitchKi = DEFAULT_PITCH_KI;
  params.pitchKd = DEFAULT_PITCH_KD;
  params.maxStabAngleDeg = DEFAULT_STAB_ANGLE_MAX;
  params.wpRadiusM = DEFAULT_WP_RADIUS_M;
  params.homeRadiusM = DEFAULT_HOME_RADIUS_M;
  params.navBankGain = DEFAULT_NAV_BANK_GAIN;
  params.rtlBankAngle = DEFAULT_RTL_BANK_DEG;
  params.autoBankAngle = DEFAULT_AUTO_BANK_DEG;
  params.rtlCruiseThrottleUs = DEFAULT_RTL_THR_US;
  params.autoCruiseThrottleUs = DEFAULT_AUTO_THR_US;
  params.rcTimeoutMs = DEFAULT_RC_TIMEOUT_MS;
}

void paramsBegin() {
  paramsLoadDefaults();
  prefs.begin("uavparams", /*readOnly=*/false);
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    float stored = prefs.getFloat(paramTable[i].name, NAN);
    if (!isnan(stored)) *(paramTable[i].value) = stored;
  }
}

void paramsSave() {
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    prefs.putFloat(paramTable[i].name, *(paramTable[i].value));
  }
}

bool paramGetByIndex(uint16_t index, const char *&name, float &value) {
  if (index >= PARAM_COUNT) return false;
  name = paramTable[index].name;
  value = *(paramTable[index].value);
  return true;
}

bool paramGetByName(const char *name, float &value) {
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    if (strncmp(paramTable[i].name, name, 16) == 0) {
      value = *(paramTable[i].value);
      return true;
    }
  }
  return false;
}

bool paramSetByName(const char *name, float value) {
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    if (strncmp(paramTable[i].name, name, 16) == 0) {
      *(paramTable[i].value) = value;
      return true;
    }
  }
  return false;
}
