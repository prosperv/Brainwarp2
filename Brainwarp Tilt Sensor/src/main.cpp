#include <Arduino.h>
#include <Wire.h>
#include "fusion.h"

// #define DEBUG
#include "debug.h"

#define SENSOR_UPDATE_RATE_HZ 100
const unsigned long DELAY_US = (SENSOR_UPDATE_RATE_HZ * 100);

#ifdef _AVR_IOM328P_H_
#include <RA4051.h>
#define ENABLE_PIN 7
RA4051 mux(6, 5, 4);
#elif ARDUINO_attiny3217
#include "RA4051.h"
#define ENABLE_PIN 10
RA4051 mux(11, 12, 13);
#endif

enum class ToySide
{
  None = 0,
  PurpleOne,
  RedTwo,
  GreenThree,
  WhiteFour,
  OrangeFive,
  YellowSix,
};

// Get toy side from orientation vector.
ToySide rotationCorrection(int vector[3])
{
  ToySide ret;
  if (vector[0] == 1 && vector[1] == 0 && vector[2] == 0)
    ret = ToySide::YellowSix;
  else if (vector[0] == -1 && vector[1] == 0 && vector[2] == 0)
    ret = ToySide::OrangeFive;
  else if (vector[0] == 0 && vector[1] == 1 && vector[2] == 0)
    ret = ToySide::PurpleOne;
  else if (vector[0] == 0 && vector[1] == -1 && vector[2] == 0)
    ret = ToySide::RedTwo;
  else if (vector[0] == 0 && vector[1] == 0 && vector[2] == 1)
    ret = ToySide::GreenThree;
  else if (vector[0] == 0 && vector[1] == 0 && vector[2] == -1)
    ret = ToySide::WhiteFour;
  else
    ret = ToySide::None;

  return ret;
}

// Set the multiplexer base on toy orientation.
void setSwitch(ToySide side)
{
  uint8_t muxValue = mux.getCurrentPin();

  mux.off();
  // arduino compiler doesn't support maps =( and I'm too lazy to add a library that supports it.
  switch (side)
  {
  case ToySide::PurpleOne:
    muxValue = 3; //3
    break;
  case ToySide::RedTwo:
    muxValue = 5; //5
    break;
  case ToySide::GreenThree:
    muxValue = 1; //1
    break;
  case ToySide::WhiteFour:
    muxValue = 7; //7
    break;
  case ToySide::OrangeFive:
    muxValue = 6; //6
    break;
  case ToySide::YellowSix:
    muxValue = 4; //4
    break;
  case ToySide::None:
    muxValue = -1;
  default:
    break;
  }
  if (muxValue != -1)
  {
    mux.setPin(muxValue);
    mux.on();
  }
}

Fusion fusion;
ToySide lastSide;
auto _lastReadTime = micros();

void setup()
{
  // initialize serial communication
#ifdef DEBUG
  Serial.begin(115200);
#endif

#ifdef _AVR_IOM328P_H_
  PRINTLN("UNO");
#else
  PRINTLN("TINYCORE");
#endif
  fusion.begin();
  mux.setEnablePin(ENABLE_PIN);
  PRINTLN("Ready player one");
}

void loop()
{
  auto readTime = micros();
  auto readDiffTime = _lastReadTime - readTime;
  if (readDiffTime >= DELAY_US)
  {
    int *imuVector = fusion.process();

    ToySide side = rotationCorrection(imuVector);

    if (side != lastSide)
    {
      PRINT("Side: ");
      PRINTLN(static_cast<int>(side));
      setSwitch(side);
      lastSide = side;
    }
    _lastReadTime = readTime;
#ifdef DEBUG
    delay(100);
#endif
  }
}