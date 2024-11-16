#pragma once
#include <Arduino.h>
#define motlf 13 // motor pins
#define motlb 12
#define motrf 27
#define motrb 26

const int freqm = 4000; // motor pwm frequency
const int mrfc = 4;     // Motor Right Forward Channel (mrfc)
const int mrbc = 5;     // Motor Right Backward Channel (mrbc)
const int mlfc = 6;     // Motor Left Forward Channel (mlfc)
const int mlbc = 7;     //
const int mpwmr = 8;    // motor pwm resolution(8bit)

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
