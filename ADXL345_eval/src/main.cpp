#include <Arduino.h>
#include <Wire.h>
#include "adxl345.h"

void setup()
{
  adxl345_setup();
}

void loop()
{
  adxl345_loop();
}
