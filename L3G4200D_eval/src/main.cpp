#include <Arduino.h>
#include <Wire.h>
#include <L3G.h>

enum class DPSrange
{
  dps250 = 0b00000000,
  dps500 = 0b00010000,
  dps2000 = 0b00011000
};

void computeScaledGyro(double scaledGyro[3], L3G::vector<int16_t> g)
{
  const double dpsScale = 2000;
  const double maxRange = 32768 - 1;
  const double extraCorrection = 1.23;
  const auto correctionFactor = dpsScale / maxRange * extraCorrection;
  scaledGyro[0] = static_cast<double>(g.x) * correctionFactor;
  scaledGyro[1] = static_cast<double>(g.y) * correctionFactor;
  scaledGyro[2] = static_cast<double>(g.z) * correctionFactor;
}

L3G gyro;

double scaledGyro[3];
double accumulatedDegree[3] = {0, 0, 0};

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

  gyro.writeReg(L3G::CTRL_REG2, 0b00000000);
  gyro.writeReg(L3G::CTRL_REG3, 0b00000000);
  gyro.writeReg(L3G::CTRL_REG4, 0b00000000);
  gyro.writeReg(L3G::CTRL_REG5, 0b00000000);
  Serial.println("Ready player one.");

  gyro.enableDefault();

  // DR = 01 (200 Hz ODR); BW = 11 (70 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
  gyro.writeReg(L3G::CTRL_REG1, 0b01101111);
  // FS = 11 (2000dps)
  gyro.writeReg(L3G::CTRL_REG4, 0b00110000);
}

void loop()
{
  const int msDelay = 20;
  auto time0 = micros();
  gyro.read();

  computeScaledGyro(scaledGyro, gyro.g);

  // Serial.print(scaledGyro[0]);
  // Serial.print(", ");
  // Serial.print(scaledGyro[1]);
  // Serial.print(", ");
  // Serial.println(scaledGyro[2]);

  accumulatedDegree[0] += scaledGyro[0] * ((msDelay + 1) / 1000.0);
  accumulatedDegree[1] += scaledGyro[1] * ((msDelay + 1) / 1000.0);
  accumulatedDegree[2] += scaledGyro[2] * ((msDelay + 1) / 1000.0);

  auto time1 = micros();
  const double zeroThreshold = 10;
  if (abs(scaledGyro[0]) < zeroThreshold && abs(scaledGyro[1]) < zeroThreshold && abs(scaledGyro[2]) < zeroThreshold)
  {
    if (abs(accumulatedDegree[0]) > zeroThreshold || abs(accumulatedDegree[1]) > zeroThreshold || abs(accumulatedDegree[2]) > zeroThreshold)
    {
      Serial.print("accum ");
      Serial.print("X: ");
      Serial.print(accumulatedDegree[0]);
      Serial.print(" Y: ");
      Serial.print(accumulatedDegree[1]);
      Serial.print(" Z: ");
      Serial.println(accumulatedDegree[2]);

      Serial.print("Time: ");
      Serial.println(time1 - time0);
      accumulatedDegree[0] = 0;
      accumulatedDegree[1] = 0;
      accumulatedDegree[2] = 0;
    }
  }

  delay(msDelay);
}