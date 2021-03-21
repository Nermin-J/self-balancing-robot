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
forward(uint8_t dutyCycle)
{
analogWrite(_pin1, dutyCycle);
analogWrite(_pin2, 0);
analogWrite(_pin3, dutyCycle);
analogWrite(_pin4, 0);
}
backward(uint8_t dutyCycle)
{
analogWrite(_pin1, 0);
analogWrite(_pin2, -dutyCycle);
analogWrite(_pin3, 0);
analogWrite(_pin4, -dutyCycle);
}
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
classCar car(3, 9, 10, 11);
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

double setPoint= 179;
double Kp = 21; //Set this first
double Kd = 0.8; //Set this secound
double Ki = 50; //Finally set this
double input, output;
PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
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
// supply your own gyro offsets here, scaled for min sensitivity
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
// enable Arduino interrupt detection
Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
attachInterrupt(0, dmpDataReady, RISING);
mpuIntStatus = mpu.getIntStatus();
// set our DMP Ready flag so the main loop() function knows it's okay to use it
Serial.println(F("DMP ready! Waiting for first interrupt..."));
dmpReady = true;
// get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize();
//setup PID
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
//no mpu data - performing PID calculations and output to motors
pid.Compute();
if (input>160 && input<200){//If the Bot is falling
if (output>0) //Falling towards front
car.forward(output); //Rotate the wheels forward
else if (output<0) //Falling towards back
car.backward(output); //Rotate the wheels backward
}
else //If Bot not falling
car.STOP(); //Hold the wheels still
}
// reset interrupt flag and get INT_STATUS byte
mpuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();
// get current FIFO count
fifoCount = mpu.getFIFOCount();
// check for overflow (this should never happen unless our code is too inefficient)
if ((mpuIntStatus & 0x10) || fifoCount == 1024)
{
// reset so we can continue cleanly
mpu.resetFIFO();
Serial.println(F("FIFO overflow!"));
// otherwise, check for DMP data ready interrupt (this should happen frequently)
}
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
input = ypr[1] * 180/M_PI + 180;
}
}
