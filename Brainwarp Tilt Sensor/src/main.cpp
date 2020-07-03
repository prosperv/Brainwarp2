#include <Arduino.h>
#include <RA4051.h>
#include <Wire.h>
#include "fusion.h"

#define DEBUG
#ifdef DEBUG
#define PRINT Serial.print
#define PRINTLN Serial.println
#else
#define PRINT
#define PRINTLN
#endif

#ifdef _AVR_IOM328P_H_
#define ENABLE_PIN 7
RA4051 mux(6, 5, 4);
#elif ARDUINO_attiny3217
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

ToySide rotationCorrection(IMUSide side)
{
  ToySide ret;
  switch (side)
  {
  case IMUSide::xMinus:
    ret = ToySide::OrangeFive;
    break;
  case IMUSide::xPlus:
    ret = ToySide::YellowSix;
    break;
  case IMUSide::yMinus:
    ret = ToySide::RedTwo;
    break;
  case IMUSide::yPlus:
    ret = ToySide::PurpleOne;
    break;
  case IMUSide::zMinus:
    ret = ToySide::WhiteFour;
    break;
  case IMUSide::zPlus:
    ret = ToySide::GreenThree;
    break;
  default:
    ret = ToySide::None;
    break;
  }
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

  IMUSide imuSide = fusion.process();
  ToySide side = rotationCorrection(imuSide);

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