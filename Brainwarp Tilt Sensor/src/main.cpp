#include <Arduino.h>
#include <Wire.h>
#include "fusion.h"

#define DEBUG
#include "debug.h"

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

void setSwitch(ToySide side)
{
  uint8_t muxValue = mux.getCurrentPin();

  mux.off();
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
    PRINT("mux: ");
    PRINTLN(muxValue);
    mux.setPin(muxValue);
    mux.on();
  }
}

Fusion fusion;
ToySide lastSide;

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
  int *imuVector = fusion.process();
  ToySide side = rotationCorrection(imuVector);

  if (side != lastSide)
  {
    PRINT("Side: ");
    PRINTLN(static_cast<int>(side));
    setSwitch(side);
    lastSide = side;
  }
#ifdef DEBUG
  delay(50);
#endif
}