/// Baancing Robot By Afnan Yusuf

/*
Accel Offsets -> X: -711 Y: -277 Z: -1067
Gyro Offsets -> X: -57 Y: -148 Z: -80
*/

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

MPU6050 mpu;
volatile bool mpuInterrupt = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
float euler[3];
void handleFIFOData();
void IRAM_ATTR dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Serial.begin(115200);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection())
  {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }

  Serial.println("Initializing DMP...");
  if (mpu.dmpInitialize() == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpu.resetFIFO();
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready!");
  }
  else
  {
    Serial.println("DMP Initialization failed");
    while (1)
      ;
  }

  mpu.setXAccelOffset(-711);
  mpu.setYAccelOffset(-277);
  mpu.setZAccelOffset(1067);
  mpu.setXGyroOffset(-57);
  mpu.setYGyroOffset(-148);
  mpu.setZGyroOffset(-80);
  mpu.setDLPFMode(0);
  mpu.setRate(200);
}

void loop()
{
  if (!mpuInterrupt)
    return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  handleFIFOData();
}

void handleFIFOData()
{
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
    return;
  }

  while (fifoCount >= packetSize)
  {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);


    Serial.println(euler[1] * 180 / M_PI);

  }
}
