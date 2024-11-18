#pragma once
#include <Arduino.h>
#include <PID_v1.h>
#define motlf 13 // motor pins
#define motlb 12
#define motrf 27
#define motrb 26

#define motl_encodea 39 // encoder pins
#define motl_encodeb 34
#define motr_encodea 35
#define motr_encodeb 32

const int freqm = 1000; // motor pwm frequency
const int mrfc = 4;     // Motor Right Forward Channel (mrfc)
const int mrbc = 5;     // Motor Right Backward Channel (mrbc)
const int mlfc = 6;     // Motor Left Forward Channel (mlfc)
const int mlbc = 7;     //
const int mpwmr = 8;    // motor pwm resolution(8bit)

volatile long encoderLeftCount = 0;  // Left motor encoder count
volatile long encoderRightCount = 0; // Right motor encoder count

double Inputleft, Outputleft, Inputright, Outputright;
double Setpointleft = 0, Setpointright = 0;

double Kpl = 1, Kil = 0, Kdl = 0;

double Kpr = 1, Kir = 0, Kdr = 0;

PID pidleft(&Inputleft, &Outputleft, &Setpointleft, Kpl, Kil, Kdl, DIRECT);

PID pidright(&Inputright, &Outputright, &Setpointright, Kpr, Kir, Kdr, DIRECT);

int leftmotspeed = 0, rightmotspeed = 0;
unsigned long lastTime = 0;
int sampletime = 20;
long deltacountleft = 0, deltacountright = 0;
double encoderLeftSpeed = 0, encoderRightSpeed = 0;

long lastLeftCount = 0, lastRightCount = 0;
long currentLeftCount = 0, currentRightCount = 0;

// Interrupt service routines for the encoders
void IRAM_ATTR handleLeftEncoderA()
{
    if (digitalRead(motl_encodea) == HIGH)
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(motl_encodeb) == LOW)
        {
            encoderLeftCount = encoderLeftCount + 1; // CW
        }
        else
        {
            encoderLeftCount = encoderLeftCount - 1; // CCW
        }
    }
    else // must be a high-to-low edge on channel A
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(motl_encodeb) == HIGH)
        {
            encoderLeftCount = encoderLeftCount + 1; // CW
        }
        else
        {
            encoderLeftCount = encoderLeftCount - 1; // CCW
        }
    }
}

void IRAM_ATTR handleRightEncoderA()
{
    if (digitalRead(motr_encodea) == HIGH)
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(motr_encodeb) == LOW)
        {
            encoderRightCount = encoderRightCount + 1; // CW
        }
        else
        {
            encoderRightCount = encoderRightCount - 1; // CCW
        }
    }
    else // must be a high-to-low edge on channel A
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(motr_encodeb) == HIGH)
        {
            encoderRightCount = encoderRightCount + 1; // CW
        }
        else
        {
            encoderRightCount = encoderRightCount - 1; // CCW
        }
    }
}

void IRAM_ATTR handleLeftEncoderB()
{
    if (digitalRead(motl_encodeb) == HIGH)
    {
        // check channel A to see which way encoder is turning
        if (digitalRead(motl_encodea) == HIGH)
        {
            encoderLeftCount = encoderLeftCount + 1; // CW
        }
        else
        {
            encoderLeftCount = encoderLeftCount - 1; // CCW
        }
    }
    // Look for a high-to-low on channel B
    else
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(motl_encodea) == LOW)
        {
            encoderLeftCount = encoderLeftCount + 1; // CW
        }
        else
        {
            encoderLeftCount = encoderLeftCount - 1; // CCW
        }
    }
}

void IRAM_ATTR handleRightEncoderB()
{
    if (digitalRead(motr_encodeb) == HIGH)
    {
        // check channel A to see which way encoder is turning
        if (digitalRead(motr_encodea) == HIGH)
        {
            encoderRightCount = encoderRightCount + 1; // CW
        }
        else
        {
            encoderRightCount = encoderRightCount - 1; // CCW
        }
    }
    // Look for a high-to-low on channel B
    else
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(motr_encodea) == LOW)
        {
            encoderRightCount = encoderRightCount + 1; // CW
        }
        else
        {
            encoderRightCount = encoderRightCount - 1; // CCW
        }
    }
}
void InitMot()
{
    ledcSetup(mrfc, freqm, mpwmr); // pwm channel and frequency and resolution setup
    ledcSetup(mrbc, freqm, mpwmr);
    ledcSetup(mlfc, freqm, mpwmr);
    ledcSetup(mlbc, freqm, mpwmr);
    ledcAttachPin(motrf, mrfc); // attach motor pins pwm channels
    ledcAttachPin(motrb, mrbc);
    ledcAttachPin(motlf, mlfc);
    ledcAttachPin(motlb, mlbc);
    ledcWrite(mrfc, 0); // write all pwmm channel 0
    ledcWrite(mrbc, 0);
    ledcWrite(mlfc, 0);
    ledcWrite(mlbc, 0);

    attachInterrupt(digitalPinToInterrupt(motl_encodea), handleLeftEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motr_encodea), handleRightEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motl_encodeb), handleLeftEncoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motr_encodeb), handleRightEncoderB, CHANGE);

    pidleft.SetMode(AUTOMATIC);
    pidright.SetMode(AUTOMATIC);
    pidleft.SetOutputLimits(-255, 255);
    pidright.SetOutputLimits(-255, 255);
}
void StopMot()
{
    ledcWrite(mrfc, 0); // write all pwmm channel 0
    ledcWrite(mrbc, 0);
    ledcWrite(mlfc, 0);
    ledcWrite(mlbc, 0);
}
void fd(int spd)
{
    ledcWrite(mrfc, spd); // write all pwmm channel 0
    ledcWrite(mrbc, 0);
    ledcWrite(mlfc, spd);
    ledcWrite(mlbc, 0);
}
void bk(int spd)
{
    ledcWrite(mrfc, 0); // write all pwmm channel 0
    ledcWrite(mrbc, spd);
    ledcWrite(mlfc, 0);
    ledcWrite(mlbc, spd);
}
void rt(int spd)
{
    ledcWrite(mrfc, spd); // write all pwmm channel 0
    ledcWrite(mrbc, 0);
    ledcWrite(mlfc, 0);
    ledcWrite(mlbc, spd);
}
void lt(int spd)
{
    ledcWrite(mrfc, 0); // write all pwmm channel 0
    ledcWrite(mrbc, spd);
    ledcWrite(mlfc, spd);
    ledcWrite(mlbc, 0);
}
void leftmotforward(int spd)
{
    ledcWrite(mrfc, spd); // write all pwmm channel 0
    ledcWrite(mrbc, 0);
}
void rightmotforward(int spd)
{
    ledcWrite(mlfc, spd); // write all pwmm channel 0
    ledcWrite(mlbc, 0);
}
void leftmotbackward(int spd)
{
    ledcWrite(mrfc, 0); // write all pwmm channel 0
    ledcWrite(mrbc, spd);
}
void rightmotbackward(int spd)

{
    ledcWrite(mlfc, 0); // write all pwmm channel 0
    ledcWrite(mlbc, spd);
}
void resetEncoders()
{
    encoderLeftCount = 0;
    encoderRightCount = 0;
}
// Function to get encoder counts
long getLeftEncoderCount()
{
    return encoderLeftCount;
}

long getRightEncoderCount()
{
    return encoderRightCount;
}

void goonencoder(int leftspeed, int rightspeed)
{
    unsigned long currentMillis = millis();

    currentLeftCount = getLeftEncoderCount();
    currentRightCount = getRightEncoderCount();

    if (currentMillis - lastTime >= sampletime)
    {
        deltacountright = currentRightCount - lastRightCount;
        deltacountleft = currentLeftCount - lastLeftCount;
        lastLeftCount = currentLeftCount;
        lastRightCount = currentRightCount;

        lastTime = currentMillis;
    }
    encoderLeftSpeed = (float)deltacountleft / ((float)sampletime / 1000.0);
    encoderRightSpeed = (float)deltacountright / ((float)sampletime / 1000.0);

    Setpointleft = leftspeed;
    Setpointright = rightspeed;

    Inputleft = encoderLeftSpeed;
    Inputright = encoderRightSpeed;

    pidleft.Compute();
    pidright.Compute();


    Serial.print(Outputleft);
    Serial.print("\t");
    Serial.println(Outputright);
}