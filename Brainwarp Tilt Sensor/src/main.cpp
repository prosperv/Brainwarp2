#include <Arduino.h>
#include <RA4051.h>
#include <Wire.h>
#include <SparkFun_ADXL345.h> // SparkFun ADXL345 Library

#define DEBUG
#ifdef DEBUG
#define PRINT Serial.print
#define PRINTLN Serial.println
#else
#define PRINT
#define PRINTLN
#endif

enum class Side
{
  None = 0,
  PurpleOne,
  RedTwo,
  GreenThree,
  WhiteFour,
  OrangeFive,
  YellowSix,
};

#ifdef _AVR_IOM328P_H_
#define ENABLE_PIN 7
RA4051 mux(6, 5, 4);
#elif ARDUINO_attiny3217
#define ENABLE_PIN 10
RA4051 mux(11, 12, 13);
#endif
// const float RANGE = 1.0;
ADXL345 accel = ADXL345(); // USE FOR I2C COMMUNICATION
Side lastSide = Side::None;

void setupTiltSensor()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(100000);
  // initialize device
  accel.powerOn();
  accel.setRangeSetting(8);
  accel.setRate(100);
}

int isOnSide(int sideToCheck, int axis1, int axis2)
{
  const int ZERO_TRESHOLD = 0.59;
  const int G_THRESHOLD = 0.78;

  // We know we're on a side if one axis has a high values while the other 2 axsi are near zero.
  // Ex. x: 0.01, y: -0.01, z: 1.1
  if (abs(sideToCheck) > G_THRESHOLD && abs(axis1) < ZERO_TRESHOLD && abs(axis2) < ZERO_TRESHOLD)
  {
    return sideToCheck > 0 ? (1) : -1;
  }
  else
  {
    return 0;
  }
};

Side calculateSide(double &ax, double &ay, double &az)
{
  Side side;

  auto mag = sqrt(sq(ax) + sq(ay) + sq(az));

  const double MAG_UPPER_LIMIT = 1.3;
  const double MAG_LOWER_LIMIT = 0.5;
  if (mag > MAG_LOWER_LIMIT && mag < MAG_UPPER_LIMIT)
  {
    int xSide = isOnSide(ax, ay, az);
    int ySide = isOnSide(ay, ax, az);
    int zSide = isOnSide(az, ay, ax);

    if (xSide == 1 && ySide == 0 && zSide == 0)
      side = Side::YellowSix;
    else if (xSide == -1 && ySide == 0 && zSide == 0)
      side = Side::OrangeFive;
    else if (xSide == 0 && ySide == 1 && zSide == 0)
      side = Side::PurpleOne;
    else if (xSide == 0 && ySide == -1 && zSide == 0)
      side = Side::RedTwo;
    else if (xSide == 0 && ySide == 0 && zSide == 1)
      side = Side::GreenThree;
    else if (xSide == 0 && ySide == 0 && zSide == -1)
      side = Side::WhiteFour;
  }
  return side;
}

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
  mux.setEnablePin(ENABLE_PIN);
  setupTiltSensor();
  PRINTLN("Ready player one");
}

void loop()
{
  double accelXYZ[3];
  accel.get_Gxyz(accelXYZ);

  Side side = calculateSide(accelXYZ[0], accelXYZ[1], accelXYZ[2]);

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