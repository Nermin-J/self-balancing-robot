#pragma once
#include <cstdint>
struct WireStub {
  bool begun = false;
  bool timedOut = false;
  uint32_t timeoutUs = 0;
  void begin() { begun = true; }
  void setWireTimeout(uint32_t timeout, bool reset) {
    timeoutUs = timeout;
    (void)reset;
  }
  bool getWireTimeoutFlag() { return timedOut; }
  void clearWireTimeoutFlag() { timedOut = false; }
};
extern WireStub Wire;
