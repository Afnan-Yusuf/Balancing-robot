/// Baancing Robot By Afnan Yusuf

/*
Accel Offsets -> X: -711 Y: -277 Z: -1067
Gyro Offsets -> X: -57 Y: -148 Z: -80
*/

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <PID_v1.h>
#include "motinit.h"

MPU6050 mpu;

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};


double Setpoint = 0, Input, Output;

int leftspeed = 0, rightspeed = 0;


//Specify the links and initial tuning parameters
float Kp=1, Ki=0, Kd=0;
PID mypid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false; 








void dmpDataReady()
{
  mpuInterrupt = true;
}

void getypr(void *parameter)
{
  while (1)
  {
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      //Serial.println(euler[1] * 180);
      
    }
  }
  vTaskDelay(1 / portTICK_PERIOD_MS);
}


void pidcontrol(void *parameter)
{
  while (1)
  {
    Input = (euler[1] * 180);
    mypid.Compute();
    rightspeed = Output;
      leftspeed = Output;
    if(Output > 0)
    {
      leftmotforward(leftspeed);
      rightmotforward(rightspeed);
    }else if (Output < 0)
    {
      leftmotbackward(-leftspeed);
      rightmotbackward(-rightspeed);

    }
     


    Serial.print(getLeftEncoderCount());
    Serial.print("\t");
    Serial.print();
    Serial.print("\t");
    Serial.println();
  }
}
void setup()
{
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.begin(115200);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(-5);
  mpu.setYGyroOffset(-37);
  mpu.setZGyroOffset(-22);
  mpu.setZAccelOffset(1688);

  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
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
  xTaskCreatePinnedToCore(getypr, "getypr", 2048, NULL, 1, NULL, 0);

  xTaskCreatePinnedToCore(pidcontrol, "pidcontrol", 2048, NULL, 1, NULL, 1);
  Input = euler[1] * 180;
  mypid.SetMode(AUTOMATIC);
  mypid.SetOutputLimits(-255, 255);
  InitMot();
}

void loop()
{
}

/**/
