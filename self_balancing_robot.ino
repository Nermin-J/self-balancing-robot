/*
 * Self-balancing robot using MPU6050 DMP pitch feedback and a PID controller.
 *
 * Data flow: MPU6050 FIFO -> quaternion/gravity -> pitch in degrees -> PID
 * -> signed motor command. Motor commands are applied while waiting for data.
 *
 * Dependencies: I2Cdev, MPU6050 MotionApps 2.0, and Arduino PID_v1.
 * Hardware: an MPU6050 on I2C, its INT signal on external interrupt 0, and
 * a motor driver with two direction inputs per motor (pins 3/9 and 10/11).
 * On an ATmega328P Uno, external interrupt 0 is digital pin 2.
 * The timer-register setup below is AVR-specific; check the board pin mapping
 * before porting. Sensor offsets, PID gains, and balance angle need calibration
 * for the assembled robot.
 */

// Drives both motors together; the pin pairs select opposite directions.
class classCar
{
private:
uint8_t _pin1;
uint8_t _pin2;
uint8_t _pin3;
uint8_t _pin4;
public:
classCar(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
_pin1 = pin1;
_pin2 = pin2;
_pin3 = pin3;
_pin4 = pin4;
pinMode(_pin1, OUTPUT);
pinMode(_pin2, OUTPUT);
pinMode(_pin3, OUTPUT);
pinMode(_pin4, OUTPUT);
}
// Apply PWM to the first input of each motor; dutyCycle is an 8-bit value.
forward(uint8_t dutyCycle)
{
analogWrite(_pin1, dutyCycle);
analogWrite(_pin2, 0);
analogWrite(_pin3, dutyCycle);
analogWrite(_pin4, 0);
}
// Apply PWM to the opposite inputs. The caller passes a negative PID output;
// this existing uint8_t interface converts it before the negation below.
backward(uint8_t dutyCycle)
{
analogWrite(_pin1, 0);
analogWrite(_pin2, -dutyCycle);
analogWrite(_pin3, 0);
analogWrite(_pin4, -dutyCycle);
}
// Set all driver inputs low; the motor driver's truth table determines braking.
STOP()
{
analogWrite(_pin1, 0);
analogWrite(_pin2, 0);
analogWrite(_pin3, 0);
analogWrite(_pin4, 0);
}
};
#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
// Motor-driver input order: motor A forward/reverse, motor B forward/reverse.
classCar car(3, 9, 10, 11);
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// Orientation decoded from each DMP packet; ypr angles are in radians.
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] angles in radians

// The loop shifts pitch by +180 degrees, so level pitch maps to 180.
// A 179-degree target represents the calibrated balance point near level.
double setPoint= 179;
double Kp = 21; // Proportional gain: tune the immediate angle correction first.
double Kd = 0.8; // Derivative gain: tune damping second.
double Ki = 50; // Integral gain: tune correction of sustained error last.
// input uses shifted pitch degrees; output is a signed PWM-scale command.
double input, output;
PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
// Interrupt handler: flag new sensor data and print the last decoded angle.
// Serial output runs inside this existing ISR; keep its timing cost in mind.
void dmpDataReady()
{
mpuInterrupt = true;
Serial.print(input);
Serial.print("\n");
}
void setup() {
Serial.begin(115200);
// initialize device
Serial.println(F("Initializing I2C devices..."));
mpu.initialize();
// verify connection
Serial.println(F("Testing device connections..."));
Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
// load and configure the DMP
devStatus = mpu.dmpInitialize();
// Sensor-specific calibration values; measure replacements for your MPU6050.
mpu.setXGyroOffset(76);
mpu.setYGyroOffset(2);
mpu.setZGyroOffset(82);
mpu.setZAccelOffset(870);
// make sure it worked (returns 0 if so)
if (devStatus == 0)
{
// turn on the DMP, now that it's ready
Serial.println(F("Enabling DMP..."));
mpu.setDMPEnabled(true);
// Listen for the MPU6050 INT rising edge on Arduino external interrupt 0.
Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
attachInterrupt(0, dmpDataReady, RISING);
mpuIntStatus = mpu.getIntStatus();
// set our DMP Ready flag so the main loop() function knows it's okay to use it
Serial.println(F("DMP ready! Waiting for first interrupt..."));
dmpReady = true;
// get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize();
// Run the PID at a 10 ms sample interval with output limited to -255..255.
pid.SetMode(AUTOMATIC);
pid.SetSampleTime(10);
pid.SetOutputLimits(-255, 255);
}
else
{
// ERROR!
// 1 = initial memory load failed

// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
Serial.print(F("DMP Initialization failed (code "));
Serial.print(devStatus);
Serial.println(F(")"));
}
// Configure AVR Timer1 and Timer2 for 8-bit fast PWM with a /8 prescaler.
// On an ATmega328P Uno these timers drive PWM pins 9/10 and 3/11.
// Both compare channels use non-inverting PWM. Changing these timers also
// affects other libraries that use them; Timer0 is left untouched.
noInterrupts();
TCCR1A = 0; //Timer/Counter Control Register 1(timer 1) A
TCCR1B = 0; //Timer/Counter Control Register 1(timer 1) B
TCCR2A = 0; //Timer/Counter Control Register 2(timer 2) A
TCCR2B = 0; //Timer/Counter Control Register 2(timer 2) B
TCCR2B |= (1<<CS21);
TCCR1B |= (1<<CS11);
TCCR2A |= 3;
TCCR1A |= 1;
TCCR1B |= (1 << WGM12);
TCCR1A |= (B00001010 << COM1B0);
TCCR2A |= (B00001010 << COM2B0);
interrupts();
car.STOP();
}
void loop() {
// if programming failed, don't try to do anything
if (!dmpReady) return;
// wait for MPU interrupt or extra packet(s) available
while (!mpuInterrupt && fifoCount < packetSize)
{
// Reuse the last decoded pitch while waiting for another sensor packet.
// Compute() updates output only when the configured sample interval elapses.
pid.Compute();
if (input>160 && input<200){// Drive only within the open 160..200 degree window.
// The output sign selects direction; a zero output leaves the previous
// motor command unchanged in this implementation.
if (output>0) //Falling towards front
car.forward(output); //Rotate the wheels forward
else if (output<0) //Falling towards back
car.backward(output); //Rotate the wheels backward
}
else // Outside the allowed tilt window, disable the motor outputs.
car.STOP(); // Set all motor-driver inputs low.
}
// reset interrupt flag and get INT_STATUS byte
mpuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();
// get current FIFO count
fifoCount = mpu.getFIFOCount();
// check for overflow (this should never happen unless our code is too inefficient)
// INT_STATUS bit 4 signals FIFO overflow; 1024 bytes is the FIFO capacity.
if ((mpuIntStatus & 0x10) || fifoCount == 1024)
{
// reset so we can continue cleanly
mpu.resetFIFO();
Serial.println(F("FIFO overflow!"));
// otherwise, check for DMP data ready interrupt (this should happen frequently)
}
// INT_STATUS bit 1 signals that a DMP packet is ready.
else if (mpuIntStatus & 0x02)
{
// wait for correct available data length, should be a VERY short wait
while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

// read a packet from FIFO
mpu.getFIFOBytes(fifoBuffer, packetSize);
// track FIFO count here in case there is > 1 packet available
// (this lets us immediately read more without waiting for an interrupt)
fifoCount -= packetSize;
mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
mpu.dmpGetGravity(&gravity, &q); //get value for gravity
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
// Convert pitch (index 1) from radians to degrees and shift by +180 to
// match the setPoint and the motor-enable window above.
input = ypr[1] * 180/M_PI + 180;
}
}
