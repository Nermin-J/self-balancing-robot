#pragma once
#include <cstdint>
#define I2CDEV_ARDUINO_WIRE 1
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
class I2Cdev {
 public:
  static int8_t readBytes(uint8_t address, uint8_t reg, uint8_t length,
                          uint8_t* data, uint16_t timeout);
};
