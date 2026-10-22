#pragma once
#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
// Inject controller outputs to exercise the sketch's actuator/fault handling.
// This double does not validate PID_v1's numerical implementation.
class PID {
 public:
  PID(double*, double* output, double*, double, double, double, int) : output_(output) {}
  int mode = MANUAL;
  int computeCalls = 0;
  int starts = 0;
  double requestedOutput = 0;
  void SetMode(int value) {
    if (value == AUTOMATIC && mode == MANUAL) ++starts;
    mode = value;
  }
  void SetSampleTime(int) {}
  void SetOutputLimits(double, double) {}
  bool Compute() {
    if (mode != AUTOMATIC) return false;
    ++computeCalls;
    *output_ = requestedOutput;
    return true;
  }
 private:
  double* output_;
};
