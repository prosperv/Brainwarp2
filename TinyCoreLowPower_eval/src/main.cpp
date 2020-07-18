#include <Arduino.h>
#include "LowPower.h"

/*
  No sleep
  5.97mA @ 3.3V

  PowerDown
  40uA @ 3.3V
  23uA @ 3.3V
  Wakes on pin change

  Standby
  34 @ 3.3V

  Idle
  2.61mA @ 3.3V
  Wakes on pin change
*/
#define POWERONOFF_PIN 15
volatile bool sleep = false;
void powerOnOff_interruptHandler()
{
  sleep = digitalRead(POWERONOFF_PIN);
}

void idle()
{
}

void setup()
{
  Serial.begin(115200);
#ifdef _AVR_IOM328P_H_
#else
  pinMode(POWERONOFF_PIN, INPUT_PULLUP);
  noInterrupts();
  attachInterrupt(POWERONOFF_PIN, powerOnOff_interruptHandler, CHANGE);
  interrupts();
#endif
}

void loop()
{
  if (sleep)
  {
    Serial.println("Going to Sleep");
    delay(10);
#if ARDUINO_attiny3217
    // enterIdle();
    // enterStandby();
    enterPowerDown();
#endif
  }
  else
  {
    Serial.println("I'm AWAKE!");
  }
  delay(100);
}