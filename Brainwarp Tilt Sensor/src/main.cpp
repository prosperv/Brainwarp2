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

int isOnSide(int a, int b, int c)
{
  const int ZERO_TRESHOLD = 38;
  const int G_THRESHOLD = 50;
  // When static, we know one axis will
  if (abs(a) > G_THRESHOLD && abs(b) < ZERO_TRESHOLD && abs(c) < ZERO_TRESHOLD)
  {
    return a > 0 ? (1) : -1;
  }
  else
  {
    return 0;
  }
};

Side calculateSide(int16_t &ax, int16_t &ay, int16_t &az)
{
  Side side;

  auto mag = sqrt(sq(ax) + sq(ay) + sq(az));

  // PRINT("mag: ");
  // PRINTLN(mag);

  const int16_t MAG_UPPER_LIMIT = 84;
  const int16_t MAG_LOWER_LIMIT = 32;
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
  int16_t ax, ay, az;
  accel.readAccel(&ax, &ay, &az);
  // read raw accel measurements from device
  // display tab-separated accel x/y/z values
  // PRINT("accel:\t");
  // PRINT(ax);
  // PRINT("\t");
  // PRINT(ay);
  // PRINT("\t");
  // PRINTLN(az);

  Side side = calculateSide(ax, ay, az);

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