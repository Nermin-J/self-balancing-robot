#include <Arduino.h>
#include <Wire.h>
#include <algorithm>
#include <iostream>
#include <limits>

uint8_t SREG = 0x80, TCCR1A = 0, TCCR1B = 0, TCCR2A = 0, TCCR2B = 0;
uint32_t clockMs = 0;
int hardwareCalls = 0;
int pinValues[20] = {};
std::vector<int> pwmHistory;
SerialStub Serial;
WireStub Wire;

// Compile and execute the actual sketch, replacing only hardware/library APIs.
#include "../self_balancing_robot.ino"

int8_t I2Cdev::readBytes(uint8_t address, uint8_t reg, uint8_t length,
                        uint8_t* data, uint16_t timeout) {
  assert(address == 0x68 && timeout == 10);
  if (mpu.failedRegister == reg) return length - 1;
  if (reg == MPU6050_RA_INT_STATUS) {
    data[0] = mpu.getIntStatus();
  } else if (reg == MPU6050_RA_FIFO_COUNTH) {
    const uint16_t count = mpu.getFIFOCount();
    data[0] = count >> 8;
    data[1] = count & 0xff;
  } else {
    assert(reg == MPU6050_RA_FIFO_R_W);
    mpu.getFIFOBytes(data, length);
  }
  return length;
}

void expectStopped() {
  if (pinValues[3] || pinValues[9] || pinValues[10] || pinValues[11]) {
    std::cerr << "Unexpected motor output at time " << clockMs
              << ", pitch " << input << ", PID output " << output << '\n';
  }
  assert(pinValues[3] == 0 && pinValues[9] == 0);
  assert(pinValues[10] == 0 && pinValues[11] == 0);
}

void reset() {
  stopController();
  mpu = MPU6050();
  Wire = WireStub();
  clockMs = 0;
  dmpReady = false;
  packetSize = 0;
  pid.requestedOutput = 0;
  pid.computeCalls = 0;
  pid.starts = 0;
  SREG = 0x80;
}

void sample(float angle, double command, uint32_t advance = 10) {
  clockMs += advance;
  mpu.packets.push_back(angle);
  pid.requestedOutput = command;
  loop();
}

int main() {
  assert(hardwareCalls == 0);  // No global-constructor I/O.
  reset();
  setup();
  assert(dmpReady && Wire.begun && Wire.timeoutUs == 3000);
  assert(SREG == 0x80);
  expectStopped();
  loop();
  assert(pid.computeCalls == 0);
  sample(179, 100);
  assert(pinValues[3] == 100 && pinValues[10] == 100);
  sample(179, -100);
  assert(pinValues[9] == 100 && pinValues[11] == 100);
  sample(179, 0);
  expectStopped();
  sample(179, 1000);
  assert(pinValues[3] == 255 && pinValues[10] == 255);
  sample(179, -1000);
  assert(pinValues[9] == 255 && pinValues[11] == 255);

  assert(!isBalanceAngle(160) && !isBalanceAngle(200));
  // Step outside boundaries to avoid float radians/degrees rounding in the fake.
  for (float angle : {159.99f, 200.01f, 150.0f, 210.0f,
                      std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::infinity()}) {
    sample(angle, 100);
    expectStopped();
    assert(pid.mode == MANUAL && output == 0);
  }
  sample(179, 30);
  assert(pid.mode == AUTOMATIC);
  const int starts = pid.starts;
  mpu.partialBytes = 1;
  loop();  // Partial packet returns immediately, without computing.
  const int computations = pid.computeCalls;
  clockMs += 100;
  loop();
  expectStopped();
  assert(pid.computeCalls == computations && pid.mode == MANUAL);
  sample(179, 30);
  assert(pid.starts == starts + 1);

  mpu.interruptStatus = 0x10;
  loop();
  expectStopped();
  assert(mpu.getFIFOCount() == 0);
  sample(179, 30);
  mpu.partialBytes = 1024;
  loop();
  expectStopped();

  sample(179, 30);
  mpu.packets = {150, 180, 179};
  const int reads = mpu.reads;
  pid.requestedOutput = -60;
  loop();
  assert(mpu.reads == reads + 3 && std::abs(input - 179) < 0.001);
  assert(pinValues[9] == 60 && pinValues[11] == 60);
  mpu.timeoutOnRead = true;
  sample(179, 40);
  expectStopped();
  mpu.timeoutOnRead = false;
  sample(179, 40);
  mpu.overflowOnRead = true;
  sample(179, 40);
  expectStopped();
  mpu.overflowOnRead = false;
  sample(179, std::numeric_limits<double>::quiet_NaN());
  expectStopped();
  Wire.timedOut = true;
  loop();
  expectStopped();

  for (int reg : {MPU6050_RA_INT_STATUS, MPU6050_RA_FIFO_COUNTH,
                  MPU6050_RA_FIFO_R_W}) {
    sample(179, 40);
    mpu.failedRegister = reg;
    sample(179, 40);
    expectStopped();
    assert(pid.mode == MANUAL);
    mpu.failedRegister = -1;
  }

  // Timeout arithmetic must continue to work across the 32-bit clock wrap.
  reset();
  setup();
  clockMs = UINT32_MAX - 50;
  sample(179, 70, 0);
  clockMs = 49;
  loop();
  expectStopped();

  for (uint16_t size : {0, 65}) {
    reset();
    mpu.size = size;
    setup();
    loop();
    assert(!dmpReady);
    expectStopped();
  }
  reset();
  mpu.connected = false;
  setup();
  loop();
  assert(!dmpReady);
  expectStopped();
  reset();
  mpu.initStatus = 1;
  setup();
  loop();
  assert(!dmpReady);
  expectStopped();
  std::cout << "All control-flow and motor-output checks passed.\n";
}
