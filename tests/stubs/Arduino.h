#pragma once
#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

#define OUTPUT 1
#define F(value) value
#define PI 3.14159265358979323846
#define _BV(bit) (1 << (bit))
enum { WGM10 = 0, WGM12 = 3, CS11 = 1, WGM20 = 0, WGM21 = 1, CS21 = 1 };
extern uint8_t SREG, TCCR1A, TCCR1B, TCCR2A, TCCR2B;
extern uint32_t clockMs;
extern int hardwareCalls;
extern int pinValues[20];
extern std::vector<int> pwmHistory;
inline uint32_t millis() { return clockMs; }
inline void noInterrupts() { SREG &= 0x7f; }
inline void pinMode(uint8_t, int) { ++hardwareCalls; }
inline void analogWrite(uint8_t pin, int value) {
  ++hardwareCalls;
  assert(value >= 0 && value <= 255);
  pinValues[pin] = value;
  pwmHistory.push_back(value);
  // Catch transient simultaneous forward/reverse drive, not just final state.
  assert(!(pinValues[3] && pinValues[9]));
  assert(!(pinValues[10] && pinValues[11]));
}
template <typename T> T constrain(T value, T low, T high) {
  return value < low ? low : value > high ? high : value;
}
struct SerialStub {
  void begin(unsigned long) {}
  template <typename T> void print(T) {}
  template <typename T> void println(T) {}
};
extern SerialStub Serial;
