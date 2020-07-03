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

void setSwitch(Side side)
{
  uint8_t muxValue = mux.getCurrentPin();

  mux.off();
  switch (side)
  {
  case Side::PurpleOne:
    muxValue = 3; //3
    break;
  case Side::RedTwo:
    muxValue = 5; //5
    break;
  case Side::GreenThree:
    muxValue = 1; //1
    break;
  case Side::WhiteFour:
    muxValue = 7; //7
    break;
  case Side::OrangeFive:
    muxValue = 6; //6
    break;
  case Side::YellowSix:
    muxValue = 4; //4
    break;
  case Side::None:
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
Side lastSide;

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

  Side side = fusion.process();
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