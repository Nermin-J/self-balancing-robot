# Self-balancing robot

Arduino Uno (ATmega328P, 16 MHz) balancing sketch using MPU6050 MotionApps 2.0
pitch feedback and PID_v1. The original motor pins, sensor offsets, angle limits,
PID gains, and PWM frequency are retained. The control loop now processes fresh
sensor packets and stops the motors on sensor faults.

## Hardware and dependencies

| Connection | Uno pin |
| --- | --- |
| Motor A forward / reverse driver inputs | 3 / 9 |
| Motor B forward / reverse driver inputs | 10 / 11 |
| MPU6050 SDA / SCL | A4 / A5 |
| MPU6050 address | `0x68` (AD0 low) |

Use a suitable motor driver and a shared ground. FIFO polling replaces the
external interrupt handler; the MPU6050 INT connection is no longer required.
Timer1 and Timer2 use fast 8-bit PWM with a /8 prescaler (approximately 7.8 kHz).
Libraries that also use these timers may conflict. Timer0 is left unchanged.

Install:

- Arduino AVR Boards **1.8.6** or a compatible core exposing Wire's
  `setWireTimeout`, `getWireTimeoutFlag`, and `clearWireTimeoutFlag` APIs.
- [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library) (`PID_v1`, 1.2.1 API).
- [I2Cdev and MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino),
  including `MPU6050_6Axis_MotionApps20.h`. Use the Arduino Wire backend.

Open `self_balancing_robot.ino` in the Arduino IDE and allow the IDE to put it
in a sketch folder named `self_balancing_robot`. Select **Arduino Uno** and verify
before uploading. Other boards require an explicit port of the pin/timer setup.

## Control behavior and calibration

Pitch is converted from radians to degrees and shifted by +180. The balance
target is 179 degrees. Motor drive is allowed only when the measured angle is
strictly between 160 and 200 degrees. Positive PID output selects the forward
inputs; negative output selects reverse with a positive PWM magnitude. Zero
output explicitly clears all four inputs. Both inputs low does not guarantee
mechanical braking; consult the motor driver's truth table.

The loop drains a bounded FIFO snapshot and uses its newest complete packet.
It never waits for an incomplete packet. PID updates use fresh samples with a
10 ms minimum computation interval. Motors remain off before the first valid
sample. Invalid angles, failed/short I2C reads, Wire timeouts, FIFO overflow, or
100 ms without a valid sample stop the motors and put the PID in manual mode.
Sensor transport faults also discard queued FIFO data. A subsequent valid,
in-range sample automatically resumes control with the PID integral reset to
zero. Initialization failures leave control disabled until reset.

Recheck sensor offsets, motor polarity, the balance target, and the original
gains (`Kp=21`, `Ki=50`, `Kd=0.8`) on the assembled robot. Updating only on fresh
measurements and fixing reverse PWM changes the original dynamics, so the old
tuning is a starting point. Validate direction and stopping with the wheels
lifted before trying to balance. The 100 ms sensor timeout also needs hardware
validation against the configured DMP output rate.

## Host regression checks

From the repository root, with a C++11-capable GCC:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -pedantic \
  -fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all \
  -D__AVR_ATmega328P__ -Itests/stubs tests/control_test.cpp -o /tmp/robot-control-test
/tmp/robot-control-test
```

These checks compile and execute the actual sketch against hardware and PID
test doubles. They cover signed PWM, saturation, zero output, direction reversal
ordering, startup failures, angle limits, non-finite input/output, partial packets,
stale data, clock rollover, FIFO draining/overflow, I2C timeouts/short reads, and
controller recovery. They do not validate PID_v1's numerical algorithm, the
actual AVR toolchain, I2C electrical behavior, or physical balance. An Uno build
and hardware checks remain necessary before deployment.
