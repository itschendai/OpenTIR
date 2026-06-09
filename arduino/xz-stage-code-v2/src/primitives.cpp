#include "primitives.h"

#include <AccelStepper.h>
#include <AS5600.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Set to 1 to test the serial protocol without moving motors.
#define SERIAL_TEST 0

// Arduino Mega 2560 target.
// D0/D1 remain the hardware serial pins for USB/serial comms.
// On the Mega 2560, I2C is on D20/D21 (SDA/SCL).
#define BLADE_RELAY  25
#define STEP1  34
#define DIR1   36
#define ENA1   38
#define STEP2  35
#define DIR2   37
#define ENA2   39
#define STEP3  10
#define DIR3   11
#define ENA3   12
#define STEP4  2
#define DIR4   3
#define ENA4   4

// Limit switches: active low with internal pull-up.
// Physical switch 2 is X home, physical switch 1 is Z home.
#define LIMIT_SWITCH_X  7
#define LIMIT_SWITCH_Z  A2
#define LIMIT_DEBOUNCE_MS  30

// HX711 force sensor interface for the vise.
#define LOADCELL_DT                  A0
#define LOADCELL_SCK                 A4
#define LOADCELL_READY_TIMEOUT_MS    180000
#define LOADCELL_COUNTS_PER_KG       (-100000.0f)
#define FORCE_SENSOR_ZERO_OFFSET_KG  0.143f
#define LOADCELL_CAPACITY_KG         50.0f
#define VISE_OPEN_MAX_KG             0.2f
#define VISE_CLOSED_MIN_KG           3.0f
#define VISE_STATE_SAMPLE_MS         10
#define AS5600_COUNTS_PER_REV        4096.0f

#define SPEED  100000.0f
#define ACCEL  10000.0f

// NEMA17 (M1, M2): 8 mm/rev, 200 full steps/rev.
#define STEPS_PER_REV_NEMA17  200.0f
#define MM_PER_REV            8.0f
#define STEPS_PER_MM          (STEPS_PER_REV_NEMA17 / MM_PER_REV)
#define MM_PER_STEP           (MM_PER_REV / STEPS_PER_REV_NEMA17)

#define HOMING_SPEED            750.0f
#define HOMING_BACKOFF_SPEED    40.0f
#define HOMING_ACCEL            10000.0f
#define SERIAL_BAUD             115200

// Rotary home is defined by the AS5600 reading 11 degrees.
#define ROTARY_HOME_DEG          11.0f
#define ROTARY_HOME_TOL_DEG      0.1f
#define ROTARY_HOME_SAFE_TOL_DEG 0.25f
#define ROTARY_HOME_SPEED        600.0f
#define ROTARY_HOME_FINE_SPEED   150.0f
#define ROTARY_MOVE_SPEED        60.0f
#define ROTARY_MOVE_FINE_SPEED   15.0f
#define CUT_HEIGHT_X_NEAR_MM     100.0f
#define CUT_HEIGHT_X_SLOW_FEED   5.0f
#define CUT_HEIGHT_ROTARY_CUT_SPEED     60.0f
#define CUT_HEIGHT_ROTARY_RETURN_SPEED  3000.0f
#define CUT_HEIGHT_TIMEOUT_MS    600000UL

// These travel limits are placeholders until the machine is measured.
#define X_MIN_MM                 0.0f
#define X_MAX_MM                 200.0f
#define Z_MIN_MM                 0.0f
#define Z_MAX_MM                 200.0f
#define LIMIT_CONTACT_ALLOW_MM   0.5f

#define HOME_TIMEOUT_MS          180000UL
#define VISE_TIMEOUT_MS          300000UL
#define VISE_CLOSE_DEFAULT_KG    4.0f
#define VISE_OPEN_DEFAULT_KG     0.2f
#define VISE_MAX_TRAVEL_STEPS    200000L
// Force at which the vise switches between fast and slow seek speed (shared by
// OPEN_VISE and CLOSE_VISE). Original value was 1.0f; bumped to 2.0f. Revert to 1.0f if needed.
#define VISE_FAST_CLOSE_THRESHOLD_KG  2.0f
#define VISE_LOW_FORCE_SPEED_STEPS_PER_SEC  5000.0f
#define VISE_CLOSE_SPEED_STEPS_PER_SEC  1000.0f
#define VISE_OPEN_SPEED_STEPS_PER_SEC   1000.0f
#define VISE_ACCEL_STEPS_PER_SEC2       40000.0f
#define VISE_GEAR_RATIO               51.0f
#define VISE_MOTOR_STEPS_PER_REV      200.0f
#define VISE_OUTPUT_STEPS_PER_REV     (VISE_MOTOR_STEPS_PER_REV * VISE_GEAR_RATIO)
#define VISE_CLUTCH_RELEASE_OUTPUT_DEG  30.0f
#define VISE_OPEN_EXTRA_OUTPUT_REV      2.0f

AccelStepper m1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper m2(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper m3(AccelStepper::DRIVER, STEP3, DIR3);
AccelStepper m4(AccelStepper::DRIVER, STEP4, DIR4);
AS5600 as5600;

enum HomingState {
  HOMING_IDLE,
  HOMING_M3,
  HOMING_M1,
  HOMING_M1_BACKOFF,
  HOMING_M2,
  HOMING_M2_BACKOFF
};

enum ViseState {
  VISE_UNKNOWN,
  VISE_OPEN,
  VISE_MOVING,
  VISE_CLOSED
};

enum ViseMotionStage {
  VISE_STAGE_IDLE,
  VISE_STAGE_SEEK_FORCE,
  VISE_STAGE_CLOSE_BACKOFF,
  VISE_STAGE_OPEN_EXTRA,
  VISE_STAGE_OPEN_RELEASE,
  VISE_STAGE_ROTARY_REHOME
};

enum CutHeightStage {
  CUT_HEIGHT_STAGE_IDLE,
  CUT_HEIGHT_STAGE_MOVE_Z,
  CUT_HEIGHT_STAGE_MOVE_X_NEAR,
  CUT_HEIGHT_STAGE_MOVE_X_CUT,
  CUT_HEIGHT_STAGE_ROTATE_FORWARD,
  CUT_HEIGHT_STAGE_MOVE_X_BACKOFF,
  CUT_HEIGHT_STAGE_ROTATE_HOME,
  CUT_HEIGHT_STAGE_MOVE_XZ_ZERO,
  CUT_HEIGHT_STAGE_HOME_ALL
};

enum ActiveCommandType {
  CMD_NONE,
  CMD_HOME_ALL,
  CMD_MOVE_X_ABS,
  CMD_MOVE_Z_ABS,
  CMD_ROTATE_ABS,
  CMD_MOVE_REL,
  CMD_CUT_HEIGHT,
  CMD_CLOSE_VISE,
  CMD_OPEN_VISE
};

struct ActiveCommand {
  ActiveCommandType type;
  long cmdId;
  char name[PROTOCOL_CMD_NAME_LEN];
  char axis;
  float target;
  float targetXMm;
  float feedOrSpeed;
  float targetForceKg;
  float cutRotaryDeg;
  float targetRotaryDeg;
  uint32_t startMs;
};

static bool as5600Connected = false;
static bool loadCellReady = false;
static bool machineHomed = false;
static bool machineFaulted = false;
static bool bladeRelayOn = false;
static ViseState viseState = VISE_UNKNOWN;
static ViseMotionStage viseMotionStage = VISE_STAGE_IDLE;
static CutHeightStage cutHeightStage = CUT_HEIGHT_STAGE_IDLE;
static HomingState homingState = HOMING_IDLE;
static ActiveCommand activeCommand = { CMD_NONE, 0, "", '\0', 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0 };

static uint32_t lastViseStateSampleMs = 0;
static uint32_t limitXPressStart = 0;
static uint32_t limitZPressStart = 0;
static uint32_t homingReleaseStart = 0;
static bool limitXLatch = false;
static bool limitZLatch = false;
static float stageX_mm = 0.0f;
static float stageZ_mm = 0.0f;
static float lastForceKg = 0.0f;
static bool lastForceKgValid = false;
static float rotaryMeasuredAngleDeg = ROTARY_HOME_DEG;
static float rotaryPositionDeg = 0.0f;
static bool rotaryPositionReferenced = false;
static uint16_t rotaryTrackedAngleDeg = (uint16_t)ROTARY_HOME_DEG;
static uint16_t rotaryMotionLastAngleDeg = (uint16_t)ROTARY_HOME_DEG;
static int8_t rotaryPositiveStepAngleSign = 0;
static int8_t rotaryMotorDir = 0;
static char faultCode[20] = "";
static char faultMessage[80] = "";

static void resetMotionProfiles();
static void syncLinearStageState();
static void syncRotaryPosition();
static void zeroRotaryPosition(float measuredDeg);
static void setBladeState(bool enabled);
static void commandLinearAxisAbsolute(char axis, float targetMm, float feedMmPerSec);
static bool updateRotaryMotionToTarget(float targetDeg, float speedDegPerSec);
static void completeActiveCommandSuccess();
static void failActiveCommand(const char* code, const char* message, bool keepHomed);
static void startRotaryHomeMotion();
static void setViseSeekSpeed(float speedStepsPerSec);
static void startCutHeight(long cmdId, float targetZMm, float targetXMm, float cutRotaryDeg);
static void updateCutHeight(uint32_t now);

static uint16_t readAS5600RawAngle() {
  return as5600.readAngle();
}

static uint16_t rawAngleToDegrees(uint16_t rawAngle) {
  return (uint16_t)(((uint32_t)rawAngle * 360UL) / 4096UL);
}

static float rawAngleToDegreesPrecise(uint16_t rawAngle) {
  return ((float)rawAngle * 360.0f) / AS5600_COUNTS_PER_REV;
}

static uint16_t readAS5600Degrees() {
  return rawAngleToDegrees(readAS5600RawAngle());
}

static float readAS5600DegreesPrecise() {
  return rawAngleToDegreesPrecise(readAS5600RawAngle());
}

static uint16_t wrapDegreesPositive(int32_t degrees) {
  int32_t value = degrees % 360;
  if (value < 0)
    value += 360;
  return (uint16_t)value;
}

static float wrapDegreesPositiveFloat(float degrees) {
  float value = fmodf(degrees, 360.0f);
  if (value < 0.0f)
    value += 360.0f;
  return value;
}

static int16_t shortestAngleErrorDeg(uint16_t currentDeg, uint16_t targetDeg) {
  int16_t error = (int16_t)targetDeg - (int16_t)currentDeg;
  while (error <= -180)
    error += 360;
  while (error > 180)
    error -= 360;
  return error;
}

static float shortestAngleErrorDegFloat(float currentDeg, float targetDeg) {
  float error = targetDeg - currentDeg;
  while (error <= -180.0f)
    error += 360.0f;
  while (error > 180.0f)
    error -= 360.0f;
  return error;
}

static void syncRotaryPositionWithMeasurement(float measuredDeg) {
  float deltaDeg = shortestAngleErrorDegFloat(rotaryMeasuredAngleDeg, measuredDeg);
  rotaryPositionDeg += deltaDeg;
  rotaryMeasuredAngleDeg = measuredDeg;
}

static void syncRotaryPosition() {
  if (!as5600Connected)
    return;
  syncRotaryPositionWithMeasurement(readAS5600DegreesPrecise());
}

static void zeroRotaryPosition(float measuredDeg) {
  rotaryMeasuredAngleDeg = measuredDeg;
  rotaryPositionDeg = 0.0f;
  rotaryPositionReferenced = true;
}

static float rotaryRelativeDeg() {
  if (!as5600Connected)
    return 0.0f;
  if (rotaryPositionReferenced)
    return rotaryPositionDeg;
  return wrapDegreesPositiveFloat(rotaryMeasuredAngleDeg - ROTARY_HOME_DEG);
}

static float loadCellRawToKg(int32_t raw) {
  return ((float)raw / LOADCELL_COUNTS_PER_KG) - FORCE_SENSOR_ZERO_OFFSET_KG;
}

static bool isLoadCellDataReady() {
  return digitalRead(LOADCELL_DT) == LOW;
}

static bool waitForLoadCellReady(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (!isLoadCellDataReady()) {
    if ((millis() - start) >= timeoutMs)
      return false;
    delay(1);
  }
  return true;
}

static bool readLoadCellRawNow(int32_t &value) {
  if (!isLoadCellDataReady())
    return false;

  uint32_t data = 0;
  noInterrupts();
  for (uint8_t i = 0; i < 24; i++) {
    digitalWrite(LOADCELL_SCK, HIGH);
    delayMicroseconds(1);
    data = (data << 1) | (uint32_t)digitalRead(LOADCELL_DT);
    digitalWrite(LOADCELL_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(LOADCELL_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(LOADCELL_SCK, LOW);
  interrupts();

  if (data & 0x800000UL)
    data |= 0xFF000000UL;
  value = (int32_t)data;
  return true;
}

static bool readLoadCellRaw(int32_t &value) {
  if (!waitForLoadCellReady(LOADCELL_READY_TIMEOUT_MS))
    return false;
  return readLoadCellRawNow(value);
}

static bool readForceKg(float &forceKg) {
  int32_t raw = 0;
  if (!loadCellReady)
    return false;
  if (!readLoadCellRaw(raw))
    return false;
  forceKg = loadCellRawToKg(raw);
  lastForceKg = forceKg;
  lastForceKgValid = true;
  return true;
}

static ViseState classifyViseState(float forceKg) {
  if (forceKg < VISE_OPEN_MAX_KG)
    return VISE_OPEN;
  if (forceKg >= VISE_CLOSED_MIN_KG)
    return VISE_CLOSED;
  return VISE_MOVING;
}

static const char* viseStateName(ViseState state) {
  switch (state) {
    case VISE_OPEN:
      return "OPEN";
    case VISE_MOVING:
      return "MOVING";
    case VISE_CLOSED:
      return "CLOSED";
    default:
      return "UNKNOWN";
  }
}

static void updateViseStateFromForceKg(float forceKg) {
  viseState = classifyViseState(forceKg);
}

static void sampleViseStateNow() {
  if (!loadCellReady)
    return;
  int32_t raw = 0;
  if (!readLoadCellRawNow(raw))
    return;
  float kg = loadCellRawToKg(raw);
  lastForceKg = kg;
  lastForceKgValid = true;
  updateViseStateFromForceKg(kg);
}

static void sampleViseStateIfDue(uint32_t now) {
  if (loadCellReady && (now - lastViseStateSampleMs) >= VISE_STATE_SAMPLE_MS) {
    lastViseStateSampleMs = now;
    sampleViseStateNow();
  }
}

static bool isXStageAtHome() {
  return fabsf(stageX_mm) <= LIMIT_CONTACT_ALLOW_MM;
}

static bool isRotaryStageAtHome() {
  if (!as5600Connected)
    return false;
  return fabsf(shortestAngleErrorDegFloat(readAS5600DegreesPrecise(), ROTARY_HOME_DEG)) <= ROTARY_HOME_SAFE_TOL_DEG;
}

static bool canOperateVise() {
  return machineHomed && isXStageAtHome() && isRotaryStageAtHome();
}

static void printStatusFields() {
  Serial.print(F(" busy="));
  protocolPrintBool(activeCommand.type != CMD_NONE);
  Serial.print(F(" homed="));
  protocolPrintBool(machineHomed);
  Serial.print(F(" faulted="));
  protocolPrintBool(machineFaulted);
}

static void printForceField() {
  Serial.print(F(" force_kg="));
  if (lastForceKgValid) {
    Serial.print(lastForceKg, 3);
  } else {
    Serial.print(F("nan"));
  }
}

static void clearFaultState() {
  machineFaulted = false;
  faultCode[0] = '\0';
  faultMessage[0] = '\0';
}

static void setFaultState(const char* code, const char* message) {
  machineFaulted = true;
  strncpy(faultCode, code, sizeof(faultCode) - 1);
  faultCode[sizeof(faultCode) - 1] = '\0';
  strncpy(faultMessage, message, sizeof(faultMessage) - 1);
  faultMessage[sizeof(faultMessage) - 1] = '\0';
}

static void syncLinearStageState() {
  stageX_mm = m1.currentPosition() * MM_PER_STEP;
  stageZ_mm = m2.currentPosition() * MM_PER_STEP;
}

static void setBladeState(bool enabled) {
  bladeRelayOn = enabled;
  digitalWrite(BLADE_RELAY, enabled ? HIGH : LOW);
}

static void stopLinearMotor(AccelStepper &motor) {
  motor.setCurrentPosition(motor.currentPosition());
}

static void stopViseMotorImmediate() {
#if !SERIAL_TEST
  long current = m4.currentPosition();
  m4.setCurrentPosition(current);
  m4.setSpeed(0.0f);
#endif
}

static void setViseSeekSpeed(float speedStepsPerSec) {
#if !SERIAL_TEST
  float speedMagnitude = (speedStepsPerSec > 0.0f) ? speedStepsPerSec : VISE_CLOSE_SPEED_STEPS_PER_SEC;
  long remaining = m4.distanceToGo();
  int direction = (remaining < 0) ? -1 : 1;
  m4.setMaxSpeed(speedMagnitude);
  m4.setSpeed(direction * speedMagnitude);
#endif
}

static void stopAllMotion() {
#if !SERIAL_TEST
  stopLinearMotor(m1);
  stopLinearMotor(m2);
  stopLinearMotor(m3);
  stopLinearMotor(m4);
#endif
  resetMotionProfiles();
  homingState = HOMING_IDLE;
}

static void resetMotionProfiles() {
  m1.setMaxSpeed(SPEED);
  m1.setAcceleration(ACCEL);
  m2.setMaxSpeed(SPEED);
  m2.setAcceleration(ACCEL);
  m3.setMaxSpeed(SPEED);
  m3.setAcceleration(ACCEL);
  m4.setMaxSpeed(SPEED);
  m4.setAcceleration(ACCEL);
}

static const char* activeCommandName() {
  return (activeCommand.type == CMD_NONE) ? "NONE" : activeCommand.name;
}

static bool isCutHeightHomeAllStage() {
  return activeCommand.type == CMD_CUT_HEIGHT && cutHeightStage == CUT_HEIGHT_STAGE_HOME_ALL;
}

static bool isViseMotionActive() {
  return activeCommand.type == CMD_CLOSE_VISE || activeCommand.type == CMD_OPEN_VISE;
}

static void clearActiveCommand() {
  activeCommand.type = CMD_NONE;
  activeCommand.cmdId = 0;
  activeCommand.name[0] = '\0';
  activeCommand.axis = '\0';
  activeCommand.target = 0.0f;
  activeCommand.targetXMm = 0.0f;
  activeCommand.feedOrSpeed = 0.0f;
  activeCommand.targetForceKg = 0.0f;
  activeCommand.cutRotaryDeg = 0.0f;
  activeCommand.targetRotaryDeg = 0.0f;
  activeCommand.startMs = 0;
  viseMotionStage = VISE_STAGE_IDLE;
  cutHeightStage = CUT_HEIGHT_STAGE_IDLE;
}

static bool beginActiveCommand(ActiveCommandType type, const char* name, long cmdId) {
  if (activeCommand.type != CMD_NONE) {
    protocolEmitError(cmdId, name, "BUSY", "Another machine command is already in progress");
    return false;
  }
  activeCommand.type = type;
  activeCommand.cmdId = cmdId;
  strncpy(activeCommand.name, name, sizeof(activeCommand.name) - 1);
  activeCommand.name[sizeof(activeCommand.name) - 1] = '\0';
  activeCommand.axis = '\0';
  activeCommand.target = 0.0f;
  activeCommand.targetXMm = 0.0f;
  activeCommand.feedOrSpeed = 0.0f;
  activeCommand.targetForceKg = 0.0f;
  activeCommand.cutRotaryDeg = 0.0f;
  activeCommand.targetRotaryDeg = 0.0f;
  activeCommand.startMs = millis();
  protocolEmitAck(cmdId, name);
  return true;
}

static bool rotarySensorRequired(long cmdId, const char* command) {
  if (as5600Connected)
    return true;
  setFaultState("SENSOR_MISSING", "AS5600 encoder is not available");
  protocolEmitError(cmdId, command, "SENSOR_MISSING", "AS5600 encoder is not available");
  return false;
}

static bool loadCellRequired(long cmdId, const char* command) {
  if (loadCellReady)
    return true;
  protocolEmitError(cmdId, command, "SENSOR_MISSING", "Load cell is not available");
  return false;
}

static bool homedRequired(long cmdId, const char* command) {
  if (machineHomed)
    return true;
  protocolEmitError(cmdId, command, "NOT_HOMED", "Machine must be homed before this command");
  return false;
}

static bool faultFreeRequired(long cmdId, const char* command) {
  if (!machineFaulted)
    return true;
  protocolEmitError(cmdId, command, "FAULTED", faultMessage[0] != '\0' ? faultMessage : "Machine is faulted");
  return false;
}

static bool inRangeFloat(float value, float minValue, float maxValue) {
  return value >= minValue && value <= maxValue;
}

static long mmToSteps(float mm) {
  return lroundf(mm * STEPS_PER_MM);
}

static long viseOutputDegreesToSteps(float degrees) {
  return lroundf((degrees / 360.0f) * VISE_OUTPUT_STEPS_PER_REV);
}

static long viseOutputRevolutionsToSteps(float revolutions) {
  return lroundf(revolutions * VISE_OUTPUT_STEPS_PER_REV);
}

static bool xLimitActive() {
  return digitalRead(LIMIT_SWITCH_X) == LOW;
}

static bool zLimitActive() {
  return digitalRead(LIMIT_SWITCH_Z) == LOW;
}

static bool xLimitContactExpected() {
  if (activeCommand.type == CMD_HOME_ALL)
    return true;
  if (activeCommand.type == CMD_MOVE_X_ABS)
    return activeCommand.target <= LIMIT_CONTACT_ALLOW_MM;
  if (activeCommand.type == CMD_MOVE_REL && activeCommand.axis == 'X')
    return activeCommand.target <= LIMIT_CONTACT_ALLOW_MM;
  if (activeCommand.type == CMD_CUT_HEIGHT && cutHeightStage == CUT_HEIGHT_STAGE_MOVE_XZ_ZERO)
    return true;
  return false;
}

static bool zLimitContactExpected() {
  if (activeCommand.type == CMD_HOME_ALL)
    return true;
  if (activeCommand.type == CMD_MOVE_Z_ABS)
    return activeCommand.target <= LIMIT_CONTACT_ALLOW_MM;
  if (activeCommand.type == CMD_MOVE_REL && activeCommand.axis == 'Z')
    return activeCommand.target <= LIMIT_CONTACT_ALLOW_MM;
  if (activeCommand.type == CMD_CUT_HEIGHT && cutHeightStage == CUT_HEIGHT_STAGE_MOVE_Z)
    return activeCommand.target <= LIMIT_CONTACT_ALLOW_MM;
  if (activeCommand.type == CMD_CUT_HEIGHT && cutHeightStage == CUT_HEIGHT_STAGE_MOVE_XZ_ZERO)
    return true;
  return false;
}

static void emitStatusSnapshot(long cmdId) {
  sampleViseStateNow();
  syncLinearStageState();
  syncRotaryPosition();
  Serial.print(F("DONE "));
  Serial.print(cmdId);
  Serial.print(F(" GET_STATUS"));
  printStatusFields();
  Serial.print(F(" x_mm="));
  Serial.print(stageX_mm, 3);
  Serial.print(F(" z_mm="));
  Serial.print(stageZ_mm, 3);
  Serial.print(F(" rot_deg="));
  Serial.print(rotaryRelativeDeg(), 3);
  Serial.print(F(" blade_on="));
  protocolPrintBool(bladeRelayOn);
  Serial.print(F(" vise_state="));
  Serial.print(viseStateName(viseState));
  printForceField();
  Serial.print(F(" x_limit="));
  protocolPrintBool(xLimitActive());
  Serial.print(F(" z_limit="));
  protocolPrintBool(zLimitActive());
  Serial.print(F(" active_command="));
  Serial.println(activeCommandName());
}

static void emitForceSnapshot(long cmdId) {
  float forceKg = 0.0f;
  if (!loadCellReady) {
    protocolEmitError(cmdId, "GET_FORCE", "SENSOR_MISSING", "Load cell is not available");
    return;
  }
  if (!readForceKg(forceKg)) {
    protocolEmitError(cmdId, "GET_FORCE", "TIMEOUT", "Load cell read timed out");
    return;
  }
  updateViseStateFromForceKg(forceKg);
  Serial.print(F("DONE "));
  Serial.print(cmdId);
  Serial.print(F(" GET_FORCE force_kg="));
  Serial.print(forceKg, 3);
  Serial.print(F(" vise_state="));
  Serial.print(viseStateName(viseState));
  printStatusFields();
  Serial.println();
}

static void completeActiveCommandSuccess() {
  syncLinearStageState();
  long cmdId = activeCommand.cmdId;
  char command[PROTOCOL_CMD_NAME_LEN];
  strncpy(command, activeCommand.name, sizeof(command) - 1);
  command[sizeof(command) - 1] = '\0';
  ActiveCommandType type = activeCommand.type;
  char axis = activeCommand.axis;

  clearActiveCommand();
  resetMotionProfiles();

  Serial.print(F("DONE "));
  Serial.print(cmdId);
  Serial.print(' ');
  Serial.print(command);

  switch (type) {
    case CMD_HOME_ALL:
      Serial.print(F(" x_mm="));
      Serial.print(stageX_mm, 3);
      Serial.print(F(" z_mm="));
      Serial.print(stageZ_mm, 3);
      Serial.print(F(" rot_deg="));
      Serial.print(rotaryRelativeDeg(), 3);
      break;
    case CMD_MOVE_X_ABS:
      Serial.print(F(" x_mm="));
      Serial.print(stageX_mm, 3);
      break;
    case CMD_MOVE_Z_ABS:
      Serial.print(F(" z_mm="));
      Serial.print(stageZ_mm, 3);
      break;
    case CMD_ROTATE_ABS:
      Serial.print(F(" rot_deg="));
      Serial.print(rotaryRelativeDeg(), 3);
      break;
    case CMD_MOVE_REL:
      Serial.print(F(" axis="));
      Serial.print(axis == 'R' ? "ROT" : (axis == 'X' ? "X" : "Z"));
      if (axis == 'X') {
        Serial.print(F(" x_mm="));
        Serial.print(stageX_mm, 3);
      } else if (axis == 'Z') {
        Serial.print(F(" z_mm="));
        Serial.print(stageZ_mm, 3);
      } else {
        Serial.print(F(" rot_deg="));
        Serial.print(rotaryRelativeDeg(), 3);
      }
      break;
    case CMD_CUT_HEIGHT:
      Serial.print(F(" x_mm="));
      Serial.print(stageX_mm, 3);
      Serial.print(F(" z_mm="));
      Serial.print(stageZ_mm, 3);
      Serial.print(F(" rot_deg="));
      Serial.print(rotaryRelativeDeg(), 3);
      Serial.print(F(" blade_on="));
      protocolPrintBool(bladeRelayOn);
      break;
    case CMD_CLOSE_VISE:
    case CMD_OPEN_VISE:
      sampleViseStateNow();
      Serial.print(F(" vise_state="));
      Serial.print(viseStateName(viseState));
      printForceField();
      break;
    default:
      break;
  }

  printStatusFields();
  Serial.println();
}

static void failActiveCommand(const char* code, const char* message, bool keepHomed) {
  long cmdId = activeCommand.cmdId;
  char command[PROTOCOL_CMD_NAME_LEN];
  strncpy(command, activeCommand.name, sizeof(command) - 1);
  command[sizeof(command) - 1] = '\0';

  stopAllMotion();
  setBladeState(false);
  clearActiveCommand();
  if (!keepHomed)
    machineHomed = false;
  setFaultState(code, message);
  protocolEmitError(cmdId, command, code, message);
}

static void commandLinearAxisAbsolute(char axis, float targetMm, float feedMmPerSec) {
#if !SERIAL_TEST
  AccelStepper* motor = (axis == 'X') ? &m1 : &m2;
  motor->setMaxSpeed((feedMmPerSec > 0.0f) ? (feedMmPerSec * STEPS_PER_MM) : SPEED);
  motor->setAcceleration(ACCEL);
  motor->moveTo(mmToSteps(targetMm));
#endif
}

static void startRotaryMotion(float targetDeg, float speedDegPerSec) {
  syncRotaryPosition();
  rotaryTrackedAngleDeg = wrapDegreesPositive((int32_t)lroundf(ROTARY_HOME_DEG + targetDeg));
  rotaryMotionLastAngleDeg = readAS5600Degrees();
  float errorDeg = targetDeg - rotaryRelativeDeg();
  rotaryMotorDir = (rotaryPositiveStepAngleSign == 0) ? 1 : ((errorDeg > 0.0f) ? rotaryPositiveStepAngleSign : -rotaryPositiveStepAngleSign);
  if (rotaryMotorDir == 0)
    rotaryMotorDir = 1;
  float speedMagnitude = (speedDegPerSec > 0.0f) ? speedDegPerSec : ROTARY_MOVE_SPEED;
#if !SERIAL_TEST
  m3.setMaxSpeed(speedMagnitude);
  m3.setAcceleration(HOMING_ACCEL);
  m3.setSpeed(rotaryMotorDir * speedMagnitude);
#endif
}

static bool updateRotaryMotionToTarget(float targetDeg, float speedDegPerSec) {
  syncRotaryPosition();
  uint16_t currentAngle = readAS5600Degrees();
  float errorDeg = targetDeg - rotaryRelativeDeg();
  if (fabsf(errorDeg) <= ROTARY_HOME_TOL_DEG) {
#if !SERIAL_TEST
    m3.setSpeed(0);
    stopLinearMotor(m3);
#endif
    return true;
  }

  int16_t angleDelta = shortestAngleErrorDeg(rotaryMotionLastAngleDeg, currentAngle);
  if (angleDelta != 0 && rotaryPositiveStepAngleSign == 0) {
    rotaryPositiveStepAngleSign = (rotaryMotorDir > 0) ? ((angleDelta > 0) ? 1 : -1)
                                                       : ((angleDelta > 0) ? -1 : 1);
  }

  if (rotaryPositiveStepAngleSign != 0)
    rotaryMotorDir = (errorDeg > 0.0f) ? rotaryPositiveStepAngleSign : -rotaryPositiveStepAngleSign;

#if !SERIAL_TEST
  float speedValue = (speedDegPerSec > 0.0f) ? speedDegPerSec : ROTARY_MOVE_SPEED;
  m3.setMaxSpeed(speedValue);
  m3.setSpeed(rotaryMotorDir * speedValue);
#endif
  rotaryMotionLastAngleDeg = currentAngle;
  return false;
}

static void startRotaryHomeMotion() {
  uint16_t rotaryRawAngle = readAS5600RawAngle();
  rotaryMotionLastAngleDeg = rawAngleToDegrees(rotaryRawAngle);
  float rotaryErrorDeg = shortestAngleErrorDegFloat(rawAngleToDegreesPrecise(rotaryRawAngle), ROTARY_HOME_DEG);

  homingState = HOMING_M3;
  homingReleaseStart = 0;
#if !SERIAL_TEST
  m3.setMaxSpeed(ROTARY_HOME_SPEED);
  m3.setAcceleration(HOMING_ACCEL);
  rotaryMotorDir = (rotaryPositiveStepAngleSign == 0) ? 1 : ((rotaryErrorDeg > 0.0f) ? rotaryPositiveStepAngleSign : -rotaryPositiveStepAngleSign);
  if (rotaryMotorDir == 0)
    rotaryMotorDir = 1;
  m3.setSpeed((rotaryPositiveStepAngleSign == 0) ? ROTARY_HOME_FINE_SPEED : (rotaryMotorDir * ROTARY_HOME_SPEED));
#endif
}

static void startHomeSequence() {
  machineHomed = false;

#if SERIAL_TEST
  m1.setCurrentPosition(0);
  m2.setCurrentPosition(0);
  zeroRotaryPosition(ROTARY_HOME_DEG);
  homingState = HOMING_IDLE;
  machineHomed = true;
  completeActiveCommandSuccess();
  return;
#endif

  // Always home rotary first from the absolute encoder before touching X/Z homing.
  startRotaryHomeMotion();
}

static void startCutHeight(long cmdId, float targetZMm, float targetXMm, float cutRotaryDeg) {
  if (!beginActiveCommand(CMD_CUT_HEIGHT, "CUT_HEIGHT", cmdId))
    return;
  setBladeState(false);
  activeCommand.target = targetZMm;
  activeCommand.targetXMm = targetXMm;
  activeCommand.cutRotaryDeg = cutRotaryDeg;
  cutHeightStage = CUT_HEIGHT_STAGE_MOVE_Z;
  commandLinearAxisAbsolute('Z', targetZMm, 0.0f);
}

static void startViseSeekMotion(bool closing, float targetForceKg) {
  activeCommand.targetForceKg = targetForceKg;
  viseMotionStage = VISE_STAGE_SEEK_FORCE;
#if !SERIAL_TEST
  m4.setMaxSpeed(closing ? VISE_LOW_FORCE_SPEED_STEPS_PER_SEC
                         : VISE_OPEN_SPEED_STEPS_PER_SEC);
  m4.setAcceleration(VISE_ACCEL_STEPS_PER_SEC2);
  m4.move(closing ? VISE_MAX_TRAVEL_STEPS : -VISE_MAX_TRAVEL_STEPS);
  setViseSeekSpeed(closing ? VISE_LOW_FORCE_SPEED_STEPS_PER_SEC
                           : VISE_OPEN_SPEED_STEPS_PER_SEC);
#endif
}

static void updateHomeSequence(uint32_t now) {
  bool cutHeightHomeAll = isCutHeightHomeAllStage();
  if (activeCommand.type != CMD_HOME_ALL && !cutHeightHomeAll)
    return;

  if ((now - activeCommand.startMs) > HOME_TIMEOUT_MS) {
    failActiveCommand("TIMEOUT", cutHeightHomeAll ? "CUT_HEIGHT final HOME_ALL timed out" : "HOME_ALL timed out", false);
    return;
  }

  if (homingState == HOMING_M3) {
    uint16_t currentRawAngle = readAS5600RawAngle();
    uint16_t currentAngle = rawAngleToDegrees(currentRawAngle);
    float currentAnglePrecise = rawAngleToDegreesPrecise(currentRawAngle);
    float errorDeg = shortestAngleErrorDegFloat(currentAnglePrecise, ROTARY_HOME_DEG);

    if (fabsf(errorDeg) <= ROTARY_HOME_TOL_DEG) {
      zeroRotaryPosition(currentAnglePrecise);
      m3.setCurrentPosition(0);
      m3.setSpeed(0);
      homingState = HOMING_M1;
      m1.setMaxSpeed(HOMING_SPEED);
      m1.setAcceleration(HOMING_ACCEL);
      m1.move(-100000L);
      return;
    }

    int16_t angleDelta = shortestAngleErrorDeg(rotaryMotionLastAngleDeg, currentAngle);
    if (angleDelta != 0 && rotaryPositiveStepAngleSign == 0) {
      rotaryPositiveStepAngleSign = (rotaryMotorDir > 0) ? ((angleDelta > 0) ? 1 : -1)
                                                         : ((angleDelta > 0) ? -1 : 1);
    }

    if (rotaryPositiveStepAngleSign != 0) {
      rotaryMotorDir = (errorDeg > 0.0f) ? rotaryPositiveStepAngleSign : -rotaryPositiveStepAngleSign;
      float speedValue = (fabsf(errorDeg) <= 8.0f) ? ROTARY_HOME_FINE_SPEED : ROTARY_HOME_SPEED;
      m3.setSpeed(rotaryMotorDir * speedValue);
    } else {
      m3.setSpeed(rotaryMotorDir * ROTARY_HOME_FINE_SPEED);
    }

    rotaryMotionLastAngleDeg = currentAngle;
  } else if (homingState == HOMING_M1) {
    if (limitXLatch) {
      stopLinearMotor(m1);
      homingState = HOMING_M1_BACKOFF;
      homingReleaseStart = 0;
      m1.setMaxSpeed(HOMING_BACKOFF_SPEED);
      m1.setAcceleration(HOMING_ACCEL);
      m1.move(100000L);
    }
  } else if (homingState == HOMING_M1_BACKOFF) {
    if (xLimitActive()) {
      homingReleaseStart = 0;
    } else if (homingReleaseStart == 0) {
      homingReleaseStart = now;
    } else if ((now - homingReleaseStart) >= LIMIT_DEBOUNCE_MS) {
      stopLinearMotor(m1);
      m1.setCurrentPosition(0);
      syncLinearStageState();
      homingState = HOMING_M2;
      homingReleaseStart = 0;
      m1.setMaxSpeed(SPEED);
      m1.setAcceleration(ACCEL);
      m2.setMaxSpeed(HOMING_SPEED);
      m2.setAcceleration(HOMING_ACCEL);
      m2.move(-100000L);
    }
  } else if (homingState == HOMING_M2) {
    if (limitZLatch) {
      stopLinearMotor(m2);
      homingState = HOMING_M2_BACKOFF;
      homingReleaseStart = 0;
      m2.setMaxSpeed(HOMING_BACKOFF_SPEED);
      m2.setAcceleration(HOMING_ACCEL);
      m2.move(100000L);
    }
  } else if (homingState == HOMING_M2_BACKOFF) {
    if (zLimitActive()) {
      homingReleaseStart = 0;
    } else if (homingReleaseStart == 0) {
      homingReleaseStart = now;
    } else if ((now - homingReleaseStart) >= LIMIT_DEBOUNCE_MS) {
      stopLinearMotor(m2);
      m2.setCurrentPosition(0);
      syncLinearStageState();
      homingState = HOMING_IDLE;
      homingReleaseStart = 0;
      machineHomed = true;
      resetMotionProfiles();
      completeActiveCommandSuccess();
    }
  }
}

static void updateRotaryClosedLoop() {
  if (activeCommand.type != CMD_ROTATE_ABS &&
      !(activeCommand.type == CMD_MOVE_REL && activeCommand.axis == 'R')) {
    return;
  }

  if (updateRotaryMotionToTarget(activeCommand.targetRotaryDeg, activeCommand.feedOrSpeed)) {
    completeActiveCommandSuccess();
  }
}

static void updateViseMotion(uint32_t now) {
  if (!isViseMotionActive())
    return;

  bool closingVise = activeCommand.type == CMD_CLOSE_VISE;

  if ((now - activeCommand.startMs) > VISE_TIMEOUT_MS) {
    failActiveCommand("TIMEOUT", "Vise motion timed out", true);
    return;
  }

  if (!loadCellReady) {
    failActiveCommand("SENSOR_MISSING", "Load cell is not available", true);
    return;
  }

  sampleViseStateIfDue(now);
  float threshold = activeCommand.targetForceKg;
  bool complete = false;
  if (viseMotionStage == VISE_STAGE_SEEK_FORCE && closingVise) {
#if !SERIAL_TEST
    float viseSpeed = (lastForceKgValid && lastForceKg < VISE_FAST_CLOSE_THRESHOLD_KG)
                      ? VISE_LOW_FORCE_SPEED_STEPS_PER_SEC
                      : VISE_CLOSE_SPEED_STEPS_PER_SEC;
    setViseSeekSpeed(viseSpeed);
#endif
    complete = lastForceKgValid && lastForceKg >= threshold;
  } else if (viseMotionStage == VISE_STAGE_SEEK_FORCE) {
#if !SERIAL_TEST
    float viseSpeed = (lastForceKgValid && lastForceKg < VISE_FAST_CLOSE_THRESHOLD_KG)
                      ? VISE_LOW_FORCE_SPEED_STEPS_PER_SEC
                      : VISE_OPEN_SPEED_STEPS_PER_SEC;
    setViseSeekSpeed(viseSpeed);
#endif
    complete = lastForceKgValid && lastForceKg <= threshold;
  }

  if (viseMotionStage == VISE_STAGE_ROTARY_REHOME) {
    uint16_t currentRawAngle = readAS5600RawAngle();
    uint16_t currentAngle = rawAngleToDegrees(currentRawAngle);
    float currentAnglePrecise = rawAngleToDegreesPrecise(currentRawAngle);
    float errorDeg = shortestAngleErrorDegFloat(currentAnglePrecise, ROTARY_HOME_DEG);

    if (fabsf(errorDeg) <= ROTARY_HOME_TOL_DEG) {
      zeroRotaryPosition(currentAnglePrecise);
#if !SERIAL_TEST
      m3.setCurrentPosition(0);
      m3.setSpeed(0);
#endif
      homingState = HOMING_IDLE;
      machineHomed = true;
      completeActiveCommandSuccess();
      return;
    }

    int16_t angleDelta = shortestAngleErrorDeg(rotaryMotionLastAngleDeg, currentAngle);
    if (angleDelta != 0 && rotaryPositiveStepAngleSign == 0) {
      rotaryPositiveStepAngleSign = (rotaryMotorDir > 0) ? ((angleDelta > 0) ? 1 : -1)
                                                         : ((angleDelta > 0) ? -1 : 1);
    }

    if (rotaryPositiveStepAngleSign != 0) {
      rotaryMotorDir = (errorDeg > 0.0f) ? rotaryPositiveStepAngleSign : -rotaryPositiveStepAngleSign;
#if !SERIAL_TEST
      float speedValue = (fabsf(errorDeg) <= 8.0f) ? ROTARY_HOME_FINE_SPEED : ROTARY_HOME_SPEED;
      m3.setSpeed(rotaryMotorDir * speedValue);
#endif
    } else {
#if !SERIAL_TEST
      m3.setSpeed(rotaryMotorDir * ROTARY_HOME_FINE_SPEED);
#endif
    }

    rotaryMotionLastAngleDeg = currentAngle;
    return;
  }

  if (viseMotionStage == VISE_STAGE_CLOSE_BACKOFF || viseMotionStage == VISE_STAGE_OPEN_RELEASE) {
    if (m4.distanceToGo() == 0) {
#if !SERIAL_TEST
      stopViseMotorImmediate();
#endif
      machineHomed = false;
      startRotaryHomeMotion();
      viseMotionStage = VISE_STAGE_ROTARY_REHOME;
    }
    return;
  }

  if (viseMotionStage == VISE_STAGE_OPEN_EXTRA) {
    if (m4.distanceToGo() == 0) {
#if !SERIAL_TEST
      stopViseMotorImmediate();
      m4.setMaxSpeed(VISE_OPEN_SPEED_STEPS_PER_SEC);
      m4.setAcceleration(VISE_ACCEL_STEPS_PER_SEC2);
      // After the large open-slack move, reverse the output shaft slightly to disengage the clutch.
      m4.move(viseOutputDegreesToSteps(VISE_CLUTCH_RELEASE_OUTPUT_DEG));
      viseMotionStage = VISE_STAGE_OPEN_RELEASE;
#else
      completeActiveCommandSuccess();
#endif
    }
    return;
  }

  if (complete) {
#if !SERIAL_TEST
    stopViseMotorImmediate();
    if (closingVise) {
      m4.setMaxSpeed(VISE_CLOSE_SPEED_STEPS_PER_SEC);
      m4.setAcceleration(VISE_ACCEL_STEPS_PER_SEC2);
      m4.move(-viseOutputDegreesToSteps(VISE_CLUTCH_RELEASE_OUTPUT_DEG));
      viseMotionStage = VISE_STAGE_CLOSE_BACKOFF;
    } else {
      m4.setMaxSpeed(VISE_LOW_FORCE_SPEED_STEPS_PER_SEC);
      m4.setAcceleration(VISE_ACCEL_STEPS_PER_SEC2);
      // Keep opening well past the low-force threshold using output-shaft revolutions.
      m4.move(-viseOutputRevolutionsToSteps(VISE_OPEN_EXTRA_OUTPUT_REV));
      viseMotionStage = VISE_STAGE_OPEN_EXTRA;
    }
#else
    stopViseMotorImmediate();
    completeActiveCommandSuccess();
#endif
    return;
  }

  if (m4.distanceToGo() == 0) {
    failActiveCommand("TIMEOUT", "Vise motion finished before reaching target force", true);
  }
}

static void updateCutHeight(uint32_t now) {
  if (activeCommand.type != CMD_CUT_HEIGHT)
    return;

  if ((now - activeCommand.startMs) > CUT_HEIGHT_TIMEOUT_MS) {
    failActiveCommand("TIMEOUT", "CUT_HEIGHT timed out", true);
    return;
  }

  switch (cutHeightStage) {
    case CUT_HEIGHT_STAGE_MOVE_Z:
      if (m2.distanceToGo() == 0) {
        cutHeightStage = CUT_HEIGHT_STAGE_MOVE_X_NEAR;
        commandLinearAxisAbsolute('X', CUT_HEIGHT_X_NEAR_MM, 0.0f);
      }
      break;
    case CUT_HEIGHT_STAGE_MOVE_X_NEAR:
      if (m1.distanceToGo() == 0) {
        setBladeState(true);
        cutHeightStage = CUT_HEIGHT_STAGE_MOVE_X_CUT;
        commandLinearAxisAbsolute('X', activeCommand.targetXMm, CUT_HEIGHT_X_SLOW_FEED);
      }
      break;
    case CUT_HEIGHT_STAGE_MOVE_X_CUT:
      if (m1.distanceToGo() == 0) {
        cutHeightStage = CUT_HEIGHT_STAGE_ROTATE_FORWARD;
        activeCommand.targetRotaryDeg = activeCommand.cutRotaryDeg;
        startRotaryMotion(activeCommand.targetRotaryDeg, CUT_HEIGHT_ROTARY_CUT_SPEED);
      }
      break;
    case CUT_HEIGHT_STAGE_ROTATE_FORWARD:
      if (updateRotaryMotionToTarget(activeCommand.targetRotaryDeg, CUT_HEIGHT_ROTARY_CUT_SPEED)) {
        cutHeightStage = CUT_HEIGHT_STAGE_MOVE_X_BACKOFF;
        commandLinearAxisAbsolute('X', CUT_HEIGHT_X_NEAR_MM, CUT_HEIGHT_X_SLOW_FEED);
      }
      break;
    case CUT_HEIGHT_STAGE_MOVE_X_BACKOFF:
      if (m1.distanceToGo() == 0) {
        setBladeState(false);
        cutHeightStage = CUT_HEIGHT_STAGE_ROTATE_HOME;
        activeCommand.targetRotaryDeg = 0.0f;
        startRotaryMotion(activeCommand.targetRotaryDeg, CUT_HEIGHT_ROTARY_RETURN_SPEED);
      }
      break;
    case CUT_HEIGHT_STAGE_ROTATE_HOME:
      if (updateRotaryMotionToTarget(activeCommand.targetRotaryDeg, CUT_HEIGHT_ROTARY_RETURN_SPEED)) {
        cutHeightStage = CUT_HEIGHT_STAGE_MOVE_XZ_ZERO;
        activeCommand.target = 0.0f;
        commandLinearAxisAbsolute('X', 0.0f, 0.0f);
        commandLinearAxisAbsolute('Z', 0.0f, 0.0f);
      }
      break;
    case CUT_HEIGHT_STAGE_MOVE_XZ_ZERO:
      if (m1.distanceToGo() == 0 && m2.distanceToGo() == 0) {
        cutHeightStage = CUT_HEIGHT_STAGE_HOME_ALL;
        activeCommand.startMs = millis();
        startHomeSequence();
      }
      break;
    default:
      break;
  }
}

static void updateLinearMotionCompletion() {
  if (activeCommand.type == CMD_MOVE_X_ABS && m1.distanceToGo() == 0) {
    completeActiveCommandSuccess();
  } else if (activeCommand.type == CMD_MOVE_Z_ABS && m2.distanceToGo() == 0) {
    completeActiveCommandSuccess();
  } else if (activeCommand.type == CMD_MOVE_REL) {
    if (activeCommand.axis == 'X' && m1.distanceToGo() == 0)
      completeActiveCommandSuccess();
    else if (activeCommand.axis == 'Z' && m2.distanceToGo() == 0)
      completeActiveCommandSuccess();
  }
}

static void handleLimitActivation(bool isXLimit) {
  if (homingState != HOMING_IDLE)
    return;
  if (activeCommand.type == CMD_NONE)
    return;

  if (isXLimit && xLimitContactExpected()) {
    m1.setCurrentPosition(0);
    syncLinearStageState();
    return;
  }
  if (!isXLimit && zLimitContactExpected()) {
    m2.setCurrentPosition(0);
    syncLinearStageState();
    return;
  }

  failActiveCommand("LIMIT_HIT", isXLimit ? "Unexpected X limit switch activation" : "Unexpected Z limit switch activation", false);
}

static void updateLimitSwitchState(uint32_t now, uint8_t pin, uint32_t &pressStart, bool &latch, bool isXLimit) {
  bool pressed = digitalRead(pin) == LOW;
  if (pressed) {
    if (pressStart == 0) {
      pressStart = now;
    } else if (!latch && (now - pressStart) >= LIMIT_DEBOUNCE_MS) {
      latch = true;
      handleLimitActivation(isXLimit);
    }
  } else {
    pressStart = 0;
    latch = false;
  }
}

static void startMoveX(long cmdId, float targetMm, float feedMmPerSec) {
  if (!beginActiveCommand(CMD_MOVE_X_ABS, "MOVE_X_ABS", cmdId))
    return;
  activeCommand.target = targetMm;
  activeCommand.feedOrSpeed = feedMmPerSec;
#if !SERIAL_TEST
  commandLinearAxisAbsolute('X', targetMm, feedMmPerSec);
#endif
}

static void startMoveZ(long cmdId, float targetMm, float feedMmPerSec) {
  if (!beginActiveCommand(CMD_MOVE_Z_ABS, "MOVE_Z_ABS", cmdId))
    return;
  activeCommand.target = targetMm;
  activeCommand.feedOrSpeed = feedMmPerSec;
#if !SERIAL_TEST
  commandLinearAxisAbsolute('Z', targetMm, feedMmPerSec);
#endif
}

static void startRotateAbs(long cmdId, float relativeDeg, float speedDegPerSec) {
  if (!beginActiveCommand(CMD_ROTATE_ABS, "ROTATE_ABS", cmdId))
    return;
  activeCommand.target = relativeDeg;
  activeCommand.feedOrSpeed = speedDegPerSec;
  activeCommand.targetRotaryDeg = relativeDeg;
  startRotaryMotion(activeCommand.targetRotaryDeg, speedDegPerSec);
}

static void startMoveRelLinear(long cmdId, char axis, float targetMm, float deltaMm, float feedMmPerSec) {
  if (!beginActiveCommand(CMD_MOVE_REL, "MOVE_REL", cmdId))
    return;
  activeCommand.axis = axis;
  activeCommand.target = targetMm;
  activeCommand.feedOrSpeed = feedMmPerSec;
#if !SERIAL_TEST
  if (axis == 'X') {
    m1.setMaxSpeed((feedMmPerSec > 0.0f) ? (feedMmPerSec * STEPS_PER_MM) : SPEED);
    m1.setAcceleration(ACCEL);
    m1.move(mmToSteps(deltaMm));
  } else {
    m2.setMaxSpeed((feedMmPerSec > 0.0f) ? (feedMmPerSec * STEPS_PER_MM) : SPEED);
    m2.setAcceleration(ACCEL);
    m2.move(mmToSteps(deltaMm));
  }
#endif
}

static void startMoveRelRotary(long cmdId, float deltaDeg, float speedDegPerSec) {
  if (!beginActiveCommand(CMD_MOVE_REL, "MOVE_REL", cmdId))
    return;
  syncRotaryPosition();
  activeCommand.axis = 'R';
  activeCommand.target = rotaryRelativeDeg() + deltaDeg;
  activeCommand.feedOrSpeed = speedDegPerSec;
  activeCommand.targetRotaryDeg = activeCommand.target;
  startRotaryMotion(activeCommand.targetRotaryDeg, speedDegPerSec);
}

static void startViseCommand(long cmdId, ActiveCommandType type, const char* name, float targetForceKg) {
  if (!beginActiveCommand(type, name, cmdId))
    return;
  startViseSeekMotion(type == CMD_CLOSE_VISE, targetForceKg);
}

void primitivesHandleCommand(const ProtocolCommand &cmd) {
  long cmdId = protocolParseCmdId(cmd);

  if (protocolEqualsIgnoreCase(cmd.name, "GET_STATUS")) {
    emitStatusSnapshot(cmdId);
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "GET_FORCE")) {
    emitForceSnapshot(cmdId);
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "CLEAR_FAULTS")) {
    if (activeCommand.type != CMD_NONE) {
      protocolEmitError(cmdId, "CLEAR_FAULTS", "BUSY", "Cannot clear faults while a command is active");
      return;
    }
    if (xLimitActive() || zLimitActive()) {
      protocolEmitError(cmdId, "CLEAR_FAULTS", "FAULTED", "A limit switch is still active");
      return;
    }
    clearFaultState();
    Serial.print(F("DONE "));
    Serial.print(cmdId);
    Serial.print(F(" CLEAR_FAULTS faulted=false busy=false homed="));
    protocolPrintBool(machineHomed);
    Serial.println();
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "STOP_ALL")) {
    protocolEmitAck(cmdId, "STOP_ALL");
    stopAllMotion();
    clearActiveCommand();
    setBladeState(false);
    Serial.print(F("DONE "));
    Serial.print(cmdId);
    Serial.print(F(" STOP_ALL blade_on=false busy=false homed="));
    protocolPrintBool(machineHomed);
    Serial.print(F(" faulted="));
    protocolPrintBool(machineFaulted);
    Serial.println();
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "SET_BLADE")) {
    char state[8];
    if (!protocolParseRequiredText(cmd, "STATE", state, sizeof(state))) {
      protocolEmitError(cmdId, "SET_BLADE", "INVALID_ARG", "state=ON or state=OFF is required");
      return;
    }
    bool turnOn = false;
    if (strcmp(state, "ON") == 0) {
      turnOn = true;
      if (!faultFreeRequired(cmdId, "SET_BLADE"))
        return;
    } else if (strcmp(state, "OFF") != 0) {
      protocolEmitError(cmdId, "SET_BLADE", "INVALID_ARG", "state must be ON or OFF");
      return;
    }
    protocolEmitAck(cmdId, "SET_BLADE");
    setBladeState(turnOn);
    Serial.print(F("DONE "));
    Serial.print(cmdId);
    Serial.print(F(" SET_BLADE blade_on="));
    protocolPrintBool(bladeRelayOn);
    printStatusFields();
    Serial.println();
    return;
  }

  if (!faultFreeRequired(cmdId, cmd.name))
    return;

  if (protocolEqualsIgnoreCase(cmd.name, "HOME_ALL")) {
    if (!rotarySensorRequired(cmdId, "HOME_ALL"))
      return;
    if (!beginActiveCommand(CMD_HOME_ALL, "HOME_ALL", cmdId))
      return;
    startHomeSequence();
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "MOVE_X_ABS")) {
    float xMm = 0.0f;
    float feed = 0.0f;
    bool feedPresent = false;
    if (!protocolParseRequiredFloat(cmd, "X_MM", xMm)) {
      protocolEmitError(cmdId, "MOVE_X_ABS", "INVALID_ARG", "x_mm is required");
      return;
    }
    if (!protocolParseOptionalFloat(cmd, "FEED", feed, feedPresent)) {
      protocolEmitError(cmdId, "MOVE_X_ABS", "INVALID_ARG", "feed must be numeric");
      return;
    }
    if (!homedRequired(cmdId, "MOVE_X_ABS"))
      return;
    if (!inRangeFloat(xMm, X_MIN_MM, X_MAX_MM)) {
      protocolEmitError(cmdId, "MOVE_X_ABS", "INVALID_ARG", "x_mm is outside configured travel limits");
      return;
    }
    startMoveX(cmdId, xMm, feedPresent ? feed : 0.0f);
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "MOVE_Z_ABS")) {
    float zMm = 0.0f;
    float feed = 0.0f;
    bool feedPresent = false;
    if (!protocolParseRequiredFloat(cmd, "Z_MM", zMm)) {
      protocolEmitError(cmdId, "MOVE_Z_ABS", "INVALID_ARG", "z_mm is required");
      return;
    }
    if (!protocolParseOptionalFloat(cmd, "FEED", feed, feedPresent)) {
      protocolEmitError(cmdId, "MOVE_Z_ABS", "INVALID_ARG", "feed must be numeric");
      return;
    }
    if (!homedRequired(cmdId, "MOVE_Z_ABS"))
      return;
    if (!inRangeFloat(zMm, Z_MIN_MM, Z_MAX_MM)) {
      protocolEmitError(cmdId, "MOVE_Z_ABS", "INVALID_ARG", "z_mm is outside configured travel limits");
      return;
    }
    startMoveZ(cmdId, zMm, feedPresent ? feed : 0.0f);
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "CUT_HEIGHT")) {
    float zMm = 0.0f;
    float xMm = 0.0f;
    float deg = 0.0f;
    float forceKg = 0.0f;
    if (!protocolParseRequiredFloat(cmd, "Z_MM", zMm)) {
      protocolEmitError(cmdId, "CUT_HEIGHT", "INVALID_ARG", "z_mm is required");
      return;
    }
    if (!protocolParseRequiredFloat(cmd, "X_MM", xMm)) {
      protocolEmitError(cmdId, "CUT_HEIGHT", "INVALID_ARG", "x_mm is required");
      return;
    }
    if (!protocolParseRequiredFloat(cmd, "DEG", deg)) {
      protocolEmitError(cmdId, "CUT_HEIGHT", "INVALID_ARG", "deg is required");
      return;
    }
    if (!homedRequired(cmdId, "CUT_HEIGHT"))
      return;
    if (!rotarySensorRequired(cmdId, "CUT_HEIGHT"))
      return;
    if (!loadCellRequired(cmdId, "CUT_HEIGHT"))
      return;
    if (!inRangeFloat(zMm, Z_MIN_MM, Z_MAX_MM)) {
      protocolEmitError(cmdId, "CUT_HEIGHT", "INVALID_ARG", "z_mm is outside configured travel limits");
      return;
    }
    if (!inRangeFloat(xMm, X_MIN_MM, X_MAX_MM)) {
      protocolEmitError(cmdId, "CUT_HEIGHT", "INVALID_ARG", "x_mm is outside configured travel limits");
      return;
    }
    if (!inRangeFloat(CUT_HEIGHT_X_NEAR_MM, X_MIN_MM, X_MAX_MM)) {
      protocolEmitError(cmdId, "CUT_HEIGHT", "INVALID_STATE", "CUT_HEIGHT near X position is outside configured travel limits");
      return;
    }
    syncRotaryPosition();
    if (fabsf(rotaryRelativeDeg()) > ROTARY_HOME_SAFE_TOL_DEG) {
      protocolEmitError(cmdId, "CUT_HEIGHT", "INVALID_STATE", "Rotary stage must be at 0 before CUT_HEIGHT");
      return;
    }
    if (!readForceKg(forceKg)) {
      protocolEmitError(cmdId, "CUT_HEIGHT", "TIMEOUT", "Load cell read timed out");
      return;
    }
    updateViseStateFromForceKg(forceKg);
    if (forceKg <= VISE_CLOSED_MIN_KG) {
      protocolEmitError(cmdId, "CUT_HEIGHT", "INVALID_STATE", "Vise must be closed above 3.0 kg before cutting");
      return;
    }
    startCutHeight(cmdId, zMm, xMm, deg);
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "ROTATE_ABS")) {
    float deg = 0.0f;
    float speedDegPerSec = 0.0f;
    bool speedPresent = false;
    if (!protocolParseRequiredFloat(cmd, "DEG", deg)) {
      protocolEmitError(cmdId, "ROTATE_ABS", "INVALID_ARG", "deg is required");
      return;
    }
    if (!protocolParseOptionalFloat(cmd, "SPEED", speedDegPerSec, speedPresent)) {
      protocolEmitError(cmdId, "ROTATE_ABS", "INVALID_ARG", "speed must be numeric");
      return;
    }
    if (!homedRequired(cmdId, "ROTATE_ABS"))
      return;
    if (!rotarySensorRequired(cmdId, "ROTATE_ABS"))
      return;
    startRotateAbs(cmdId, deg, speedPresent ? speedDegPerSec : 0.0f);
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "MOVE_REL")) {
    char axis[8];
    float delta = 0.0f;
    float feed = 0.0f;
    bool feedPresent = false;
    if (!protocolParseRequiredText(cmd, "AXIS", axis, sizeof(axis))) {
      protocolEmitError(cmdId, "MOVE_REL", "INVALID_ARG", "axis is required");
      return;
    }
    if (!protocolParseRequiredFloat(cmd, "DELTA", delta)) {
      protocolEmitError(cmdId, "MOVE_REL", "INVALID_ARG", "delta is required");
      return;
    }
    if (!protocolParseOptionalFloat(cmd, "FEED", feed, feedPresent)) {
      protocolEmitError(cmdId, "MOVE_REL", "INVALID_ARG", "feed must be numeric");
      return;
    }
    if (!homedRequired(cmdId, "MOVE_REL"))
      return;
    if (strcmp(axis, "X") == 0) {
      float target = stageX_mm + delta;
      if (!inRangeFloat(target, X_MIN_MM, X_MAX_MM)) {
        protocolEmitError(cmdId, "MOVE_REL", "INVALID_ARG", "Relative X move exceeds configured travel limits");
        return;
      }
      startMoveRelLinear(cmdId, 'X', target, delta, feedPresent ? feed : 0.0f);
      return;
    }
    if (strcmp(axis, "Z") == 0) {
      float target = stageZ_mm + delta;
      if (!inRangeFloat(target, Z_MIN_MM, Z_MAX_MM)) {
        protocolEmitError(cmdId, "MOVE_REL", "INVALID_ARG", "Relative Z move exceeds configured travel limits");
        return;
      }
      startMoveRelLinear(cmdId, 'Z', target, delta, feedPresent ? feed : 0.0f);
      return;
    }
    if (strcmp(axis, "ROT") == 0) {
      if (!rotarySensorRequired(cmdId, "MOVE_REL"))
        return;
      startMoveRelRotary(cmdId, delta, feedPresent ? feed : 0.0f);
      return;
    }
    protocolEmitError(cmdId, "MOVE_REL", "INVALID_ARG", "axis must be X, Z, or ROT");
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "CLOSE_VISE")) {
    float targetForceKg = VISE_CLOSE_DEFAULT_KG;
    bool present = false;
    if (!protocolParseOptionalFloat(cmd, "TARGET_FORCE_KG", targetForceKg, present)) {
      protocolEmitError(cmdId, "CLOSE_VISE", "INVALID_ARG", "target_force_kg must be numeric");
      return;
    }
    if (!loadCellRequired(cmdId, "CLOSE_VISE"))
      return;
    if (!canOperateVise()) {
      protocolEmitError(cmdId, "CLOSE_VISE", "INVALID_STATE", "Vise requires X and rotary stages at home");
      return;
    }
    if (targetForceKg <= 0.0f || targetForceKg > LOADCELL_CAPACITY_KG) {
      protocolEmitError(cmdId, "CLOSE_VISE", "INVALID_ARG", "target_force_kg is outside the valid range");
      return;
    }
    startViseCommand(cmdId, CMD_CLOSE_VISE, "CLOSE_VISE", targetForceKg);
    return;
  }

  if (protocolEqualsIgnoreCase(cmd.name, "OPEN_VISE")) {
    float targetForceKg = VISE_OPEN_DEFAULT_KG;
    bool present = false;
    if (!protocolParseOptionalFloat(cmd, "TARGET_FORCE_KG", targetForceKg, present)) {
      protocolEmitError(cmdId, "OPEN_VISE", "INVALID_ARG", "target_force_kg must be numeric");
      return;
    }
    if (!loadCellRequired(cmdId, "OPEN_VISE"))
      return;
    if (!canOperateVise()) {
      protocolEmitError(cmdId, "OPEN_VISE", "INVALID_STATE", "Vise requires X and rotary stages at home");
      return;
    }
    if (targetForceKg < 0.0f || targetForceKg > LOADCELL_CAPACITY_KG) {
      protocolEmitError(cmdId, "OPEN_VISE", "INVALID_ARG", "target_force_kg is outside the valid range");
      return;
    }
    startViseCommand(cmdId, CMD_OPEN_VISE, "OPEN_VISE", targetForceKg);
    return;
  }

  protocolEmitError(cmdId, cmd.name, "INVALID_ARG", "Unknown command");
}

void primitivesSetup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) { }

  Wire.begin();
  as5600.begin();
  as5600Connected = as5600.isConnected();
  if (as5600Connected) {
    uint16_t initialRotaryRawAngle = readAS5600RawAngle();
    rotaryMeasuredAngleDeg = rawAngleToDegreesPrecise(initialRotaryRawAngle);
    rotaryMotionLastAngleDeg = rawAngleToDegrees(initialRotaryRawAngle);
  }

  pinMode(LOADCELL_DT, INPUT);
  pinMode(LOADCELL_SCK, OUTPUT);
  digitalWrite(LOADCELL_SCK, LOW);

  int32_t initialLoadCellRaw = 0;
  loadCellReady = readLoadCellRaw(initialLoadCellRaw);
  if (loadCellReady) {
    lastForceKg = loadCellRawToKg(initialLoadCellRaw);
    lastForceKgValid = true;
    updateViseStateFromForceKg(lastForceKg);
  }

  pinMode(LIMIT_SWITCH_X, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Z, INPUT_PULLUP);
  pinMode(BLADE_RELAY, OUTPUT);
  digitalWrite(BLADE_RELAY, LOW);

  pinMode(ENA1, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(ENA3, OUTPUT);
  pinMode(ENA4, OUTPUT);
  digitalWrite(ENA1, LOW);
  digitalWrite(ENA2, LOW);
  digitalWrite(ENA3, LOW);
  digitalWrite(ENA4, LOW);

  m1.setPinsInverted(true, false, false);
  m2.setPinsInverted(true, false, false);
  m3.setPinsInverted(true, false, false);
  m4.setPinsInverted(true, false, false);
  resetMotionProfiles();
  syncLinearStageState();

  Serial.print(F("READY protocol=primitive_api as5600="));
  protocolPrintBool(as5600Connected);
  Serial.print(F(" loadcell="));
  protocolPrintBool(loadCellReady);
  Serial.println();
}

void primitivesUpdate() {
  uint32_t now = millis();
  bool viseCommandActive = isViseMotionActive();
  bool viseSeekActive = viseCommandActive && viseMotionStage == VISE_STAGE_SEEK_FORCE;

  // Fast vise seek needs tight step timing; service M4 before sensor housekeeping.
  if (viseSeekActive) {
    m4.runSpeedToPosition();
  }

  updateLimitSwitchState(now, LIMIT_SWITCH_X, limitXPressStart, limitXLatch, true);
  updateLimitSwitchState(now, LIMIT_SWITCH_Z, limitZPressStart, limitZLatch, false);

  if (!viseCommandActive) {
    sampleViseStateIfDue(now);
  }

  // Rotary is stationary during vise motion; avoid AS5600 I2C reads that cap M4 step rate.
  if (!viseCommandActive) {
    syncRotaryPosition();
  }
  updateHomeSequence(now);
  updateRotaryClosedLoop();
  updateViseMotion(now);
  updateCutHeight(now);
  updateLinearMotionCompletion();

  m1.run();
  m2.run();
  if (homingState == HOMING_M3 ||
      activeCommand.type == CMD_ROTATE_ABS ||
      (activeCommand.type == CMD_MOVE_REL && activeCommand.axis == 'R') ||
      (activeCommand.type == CMD_CUT_HEIGHT &&
       (cutHeightStage == CUT_HEIGHT_STAGE_ROTATE_FORWARD ||
        cutHeightStage == CUT_HEIGHT_STAGE_ROTATE_HOME))) {
    m3.runSpeed();
  } else {
    m3.run();
  }
  if (isViseMotionActive() && viseMotionStage == VISE_STAGE_SEEK_FORCE) {
    m4.runSpeedToPosition();
  } else {
    m4.run();
  }

  syncLinearStageState();
}
