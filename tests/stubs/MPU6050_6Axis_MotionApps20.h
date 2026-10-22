#pragma once
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <Wire.h>
#define MPU6050_RA_INT_STATUS 0x3a
#define MPU6050_RA_FIFO_COUNTH 0x72
#define MPU6050_RA_FIFO_R_W 0x74
struct Quaternion {};
struct VectorFloat {};
class MPU6050 {
 public:
  explicit MPU6050(uint8_t = 0x68) {}
  bool connected = true;
  uint8_t initStatus = 0;
  uint16_t size = 42;
  uint16_t partialBytes = 0;
  uint8_t interruptStatus = 0;
  bool timeoutOnRead = false;
  bool overflowOnRead = false;
  int failedRegister = -1;
  int reads = 0;
  int resets = 0;
  float decodedAngle = 180;
  std::deque<float> packets;
  void initialize() { assert(Wire.begun); }
  bool testConnection() { return connected; }
  uint8_t dmpInitialize() { return initStatus; }
  void setXGyroOffset(int) {}
  void setYGyroOffset(int) {}
  void setZGyroOffset(int) {}
  void setZAccelOffset(int) {}
  uint16_t dmpGetFIFOPacketSize() { return size; }
  void setDMPEnabled(bool) {}
  void resetFIFO() {
    ++resets;
    packets.clear();
    partialBytes = 0;
    interruptStatus = 0;
  }
  uint8_t getIntStatus() { return interruptStatus; }
  uint16_t getFIFOCount() { return packets.size() * size + partialBytes; }
  void getFIFOBytes(uint8_t*, uint16_t count) {
    assert(count == size && !packets.empty());
    decodedAngle = packets.front();
    packets.pop_front();
    ++reads;
    if (timeoutOnRead) Wire.timedOut = true;
    if (overflowOnRead) interruptStatus = 0x10;
  }
  void dmpGetQuaternion(Quaternion*, uint8_t*) {}
  void dmpGetGravity(VectorFloat*, Quaternion*) {}
  void dmpGetYawPitchRoll(float* angles, Quaternion*, VectorFloat*) {
    angles[1] = (decodedAngle - 180.0f) * PI / 180.0f;
  }
};
