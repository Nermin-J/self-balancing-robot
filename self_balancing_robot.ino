#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

// Pin mapping and direct timer configuration below target the ATmega328P Uno.
#if !defined(__AVR_ATmega328P__)
#error "Select an ATmega328P Arduino Uno or port the motor pins and PWM timers."
#endif
#if I2CDEV_IMPLEMENTATION != I2CDEV_ARDUINO_WIRE
#error "Use I2Cdev's Arduino Wire backend so I2C timeouts apply."
#endif

namespace {
constexpr uint8_t kMotorAForward = 3;
constexpr uint8_t kMotorAReverse = 9;
constexpr uint8_t kMotorBForward = 10;
constexpr uint8_t kMotorBReverse = 11;
constexpr int kMaxPwm = 255;
constexpr uint16_t kFifoCapacity = 1024;
constexpr uint8_t kFifoOverflowMask = 0x10;
constexpr uint32_t kSampleTimeMs = 10;
constexpr uint32_t kSensorTimeoutMs = 100;
constexpr uint32_t kWireTimeoutUs = 3000;
constexpr uint16_t kI2cReadTimeoutMs = 10;
constexpr uint8_t kMpuAddress = 0x68;  // AD0 low; change if wired high.
constexpr double kMinAngle = 160.0;
constexpr double kMaxAngle = 200.0;
constexpr double kKp = 21.0;
constexpr double kKi = 50.0;
constexpr double kKd = 0.8;
constexpr double kRadiansToDegrees = 180.0 / PI;

class MotorDriver {
 public:
  // Hardware access belongs in setup(), after the Arduino core initializes.
  void begin() {
    pinMode(kMotorAForward, OUTPUT);
    pinMode(kMotorAReverse, OUTPUT);
    pinMode(kMotorBForward, OUTPUT);
    pinMode(kMotorBReverse, OUTPUT);
    stop();
  }

  void stop() {
    analogWrite(kMotorAForward, 0);
    analogWrite(kMotorAReverse, 0);
    analogWrite(kMotorBForward, 0);
    analogWrite(kMotorBReverse, 0);
  }

  void drive(int command) {
    command = constrain(command, -kMaxPwm, kMaxPwm);
    if (command > 0) {
      // Disable the opposite direction before enabling PWM on either motor.
      analogWrite(kMotorAReverse, 0);
      analogWrite(kMotorBReverse, 0);
      analogWrite(kMotorAForward, command);
      analogWrite(kMotorBForward, command);
    } else if (command < 0) {
      analogWrite(kMotorAForward, 0);
      analogWrite(kMotorBForward, 0);
      analogWrite(kMotorAReverse, -command);
      analogWrite(kMotorBReverse, -command);
    } else {
      stop();
    }
  }
};

MotorDriver motors;
MPU6050 mpu(kMpuAddress);
double setPoint = 179.0;  // Pitch in degrees plus 180; calibrate for the robot.
double input = 0.0;
double output = 0.0;
PID pid(&input, &output, &setPoint, kKp, kKi, kKd, DIRECT);
bool dmpReady = false;
bool controllerActive = false;
bool haveSample = false;
uint16_t packetSize = 0;
uint8_t fifoBuffer[64] = {};
uint32_t lastSampleMs = 0;

bool isBalanceAngle(double angle) {
  return isfinite(angle) && angle > kMinAngle && angle < kMaxAngle;
}

void stopController() {
  motors.stop();
  pid.SetMode(MANUAL);
  output = 0.0;
  controllerActive = false;
  haveSample = false;
}

void configurePwm() {
  // Preserve the original fast PWM /8 settings (about 7.8 kHz at 16 MHz).
  // Save/restore interrupt state; leave Timer0 (millis/delay) untouched.
  const uint8_t savedStatus = SREG;
  noInterrupts();
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS11);
  TCCR2A = _BV(WGM20) | _BV(WGM21);
  TCCR2B = _BV(CS21);
  // analogWrite enables the appropriate non-inverting compare channel.
  SREG = savedStatus;
}

bool wireTimedOut() {
  if (!Wire.getWireTimeoutFlag()) return false;
  Wire.clearWireTimeoutFlag();
  return true;
}

bool readSensorBytes(uint8_t reg, uint8_t* data, uint8_t length) {
  // MPU6050 getFIFOBytes/getFIFOCount discard the underlying read result.
  // Check it explicitly so a disconnect or short read cannot reuse old bytes.
  const int8_t count = I2Cdev::readBytes(kMpuAddress, reg, length, data,
                                        kI2cReadTimeoutMs);
  const bool timeout = wireTimedOut();
  return count == length && !timeout;
}

void discardSensorData() {
  stopController();
  mpu.resetFIFO();
  Wire.clearWireTimeoutFlag();
}
}  // namespace

void setup() {
  motors.begin();
  configurePwm();
  motors.stop();
  Serial.begin(115200);
  Wire.begin();
  // Requires the timeout API in Arduino AVR Boards 1.8.6 (or compatible).
  Wire.setWireTimeout(kWireTimeoutUs, true);
  Wire.clearWireTimeoutFlag();
  mpu.initialize();
  if (!mpu.testConnection() || wireTimedOut()) {
    Serial.println(F("MPU6050 connection failed; motors disabled."));
    return;
  }

  const uint8_t status = mpu.dmpInitialize();
  if (status != 0 || wireTimedOut()) {
    Serial.print(F("DMP initialization failed; status: "));
    Serial.println(status);
    return;
  }
  // Original calibration values; replace with measurements for your sensor.
  mpu.setXGyroOffset(76);
  mpu.setYGyroOffset(2);
  mpu.setZGyroOffset(82);
  mpu.setZAccelOffset(870);
  packetSize = mpu.dmpGetFIFOPacketSize();
  if (packetSize == 0 || packetSize > sizeof(fifoBuffer) || wireTimedOut()) {
    Serial.println(F("Invalid DMP packet size or I2C timeout; motors disabled."));
    return;
  }
  mpu.setDMPEnabled(true);
  mpu.resetFIFO();
  if (wireTimedOut()) return;

  pid.SetSampleTime(kSampleTimeMs);
  pid.SetOutputLimits(-kMaxPwm, kMaxPwm);
  // Stay in MANUAL until the first valid, in-range pitch sample arrives.
  dmpReady = true;
  Serial.println(F("DMP ready; waiting for valid pitch data."));
}

void loop() {
  if (!dmpReady) {
    stopController();
    return;
  }

  // Unsigned subtraction also works across the millis() rollover.
  if (haveSample && static_cast<uint32_t>(millis() - lastSampleMs) >= kSensorTimeoutMs) {
    discardSensorData();
    return;
  }

  // Poll FIFO directly: no serial work in an ISR and no dependency on INT wiring.
  uint8_t status = 0;
  uint8_t countBytes[2] = {};
  if (!readSensorBytes(MPU6050_RA_INT_STATUS, &status, 1) ||
      !readSensorBytes(MPU6050_RA_FIFO_COUNTH, countBytes, 2)) {
    discardSensorData();
    return;
  }
  uint16_t fifoCount = (static_cast<uint16_t>(countBytes[0]) << 8) | countBytes[1];
  if ((status & kFifoOverflowMask) || fifoCount >= kFifoCapacity) {
    discardSensorData();
    return;
  }
  // Never busy-wait for a partial packet. The timeout above still runs.
  if (fifoCount < packetSize) return;

  // Drain only this bounded snapshot; decode the newest complete packet.
  while (fifoCount >= packetSize) {
    if (!readSensorBytes(MPU6050_RA_FIFO_R_W, fifoBuffer,
                         static_cast<uint8_t>(packetSize))) {
      discardSensorData();
      return;
    }
    fifoCount -= packetSize;
  }
  uint8_t afterReadStatus = 0;
  if (!readSensorBytes(MPU6050_RA_INT_STATUS, &afterReadStatus, 1) ||
      (afterReadStatus & kFifoOverflowMask)) {
    discardSensorData();
    return;
  }

  Quaternion orientation;
  VectorFloat gravity;
  float angles[3];
  mpu.dmpGetQuaternion(&orientation, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &orientation);
  mpu.dmpGetYawPitchRoll(angles, &orientation, &gravity);
  input = angles[1] * kRadiansToDegrees + 180.0;
  if (!isBalanceAngle(input)) {
    stopController();
    return;
  }

  lastSampleMs = millis();
  haveSample = true;
  if (!controllerActive) {
    // MANUAL -> AUTOMATIC seeds the integral from zero and derivative from input.
    output = 0.0;
    pid.SetMode(AUTOMATIC);
    controllerActive = true;
  }
  pid.Compute();
  if (!isfinite(output)) {
    stopController();
    return;
  }
  // Clamp while still floating-point, then convert to a signed PWM command.
  motors.drive(static_cast<int>(constrain(output, -static_cast<double>(kMaxPwm),
                                         static_cast<double>(kMaxPwm))));
}
