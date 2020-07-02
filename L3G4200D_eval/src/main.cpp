#include <Arduino.h>
#include <Wire.h>
#include <L3G.h>

L3G gyro;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  if (!gyro.init(L3G::device_4200D))
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1)
      ;
  }

  gyro.enableDefault();
}

void loop()
{
  gyro.read();

  Serial.print("G ");
  Serial.print("X: ");
  Serial.print((int)gyro.g.x);
  Serial.print(" Y: ");
  Serial.print((int)gyro.g.y);
  Serial.print(" Z: ");
  Serial.println((int)gyro.g.z);

  delay(200);
}