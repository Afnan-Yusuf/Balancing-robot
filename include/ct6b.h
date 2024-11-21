#pragma once
#include <Arduino.h>
#include "macros.h"
void calibratect6b();
#define NUM_CHANNELS 4
int calres = 500;
int calfreq = 20;
bool calibrationstatus = true;
#define _DEBUG_
unsigned long now = 0;
unsigned long lastconnect = 0;
unsigned long connectiontimeoutdelay = 100;
bool disconnected = false;

const uint8_t channelPins[NUM_CHANNELS] = {4, 16, 17, 18}; // GPIO pins for the 6 channels
volatile uint16_t pwmValues[NUM_CHANNELS] = {0};                    // Array to store PWM values
volatile uint32_t pulseStart[NUM_CHANNELS] = {0};                   // Array to store pulse start times
void handleInterrupt(uint8_t channel);
void handleInterrupt5();
void handleInterrupt4();
void handleInterrupt3();
void handleInterrupt2();
void handleInterrupt1();
void handleInterrupt0();
void (*handleInterrupts[NUM_CHANNELS])(void) = {
    handleInterrupt0,
    handleInterrupt1,
    handleInterrupt2,
    handleInterrupt3};

int minvalues[6] = {1500, 1500, 1500, 1500, 1500, 1500};
int maxvalues[6] = {500, 500, 500, 500, 500, 500};

const int ch2max = 1995;
const int ch2min = 1195;
const int ch1max = 1953;
const int ch1min = 1023;
const int ch3max = 1911;
const int ch3min = 1105;


#define NUM_SAMPLES 50 // Number of samples for averaging
float samples[NUM_SAMPLES]; // Array to store samples
uint8_t sampleIndex = 0;    // Index for storing the current sample
bool bufferFull = false;  

float x, y, z;

float zz;


float updateFilter(float newValue) {
    // Add the new value to the samples array
    samples[sampleIndex] = newValue;
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES; // Increment index circularly

    // Determine the number of samples to average
    uint8_t count = bufferFull ? NUM_SAMPLES : sampleIndex;

    // Calculate the average of available samples
    float sum = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum += samples[i];
    }

    // Mark buffer as full if we've wrapped around
    if (sampleIndex == 0) bufferFull = true;

    return sum / count;
}





void ct6binit()
{
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        pinMode(channelPins[i], INPUT);
        attachInterrupt(channelPins[i], (void (*)(void))handleInterrupts[i], CHANGE);
    }
}
void runonct6b()
{

    now = millis();
    now - lastconnect > connectiontimeoutdelay ? disconnected = true : disconnected = false;
    z = map(pwmValues[2], ch1min, ch2max, 0, 200) - 100;
    x = map(pwmValues[0], ch1min, ch1max, 0, 510) - 255;
    y = map(pwmValues[1], ch3min, ch3max, 0, 300) - 150;
    zz = updateFilter(y) /10;

    // Serial.print(x);
    // Serial.print("\t");
    // Serial.print(y);
    // Serial.print("\t");
    // Serial.print(zz);
    // Serial.print("\t");
    // Serial.print(pwmValues[0]);
    // Serial.print("\t");
    // Serial.print(pwmValues[1]);
    // Serial.print("\t");
    // Serial.print(pwmValues[2]);
    // Serial.print("\t");
    // Serial.println(pwmValues[3]);

}

void handleInterrupt0() { handleInterrupt(0); }
void handleInterrupt1() { handleInterrupt(1); }
void handleInterrupt2() { handleInterrupt(2); }
void handleInterrupt3() { handleInterrupt(3); }
void handleInterrupt4() { handleInterrupt(4); }
void handleInterrupt5() { handleInterrupt(5); }



void handleInterrupt(uint8_t channel)
{
  uint32_t currentTime = micros();

  if (digitalRead(channelPins[channel]) == HIGH)
  {
    pulseStart[channel] = currentTime; // Store the start time of the pulse
  }
  else
  {
    pwmValues[channel] = currentTime - pulseStart[channel]; // Calculate the pulse width
    lastconnect = now;
  }
}

void calibratect6b()
{
  if (calibrationstatus == true)
  {
    for (int j = 0; j < calres; j++)
    {
      delay(calfreq);
      for (uint8_t i = 0; i < NUM_CHANNELS; i++)
      {
        pwmValues[i] > maxvalues[i] ? maxvalues[i] = pwmValues[i] : maxvalues[i] = maxvalues[i];
        pwmValues[i] < minvalues[i] ? minvalues[i] = pwmValues[i] : minvalues[i] = minvalues[i];
      }
    }
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      _PP(maxvalues[i]);
      _PP("\t");
    }
    _PL("");
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      _PP(minvalues[i]);
      _PP("\t");
    }
    delay(2000);
    calibrationstatus = false;
  }
}