#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "ADXL345.h"

#define DEBUG
#ifdef DEBUG
#define PRINT Serial.print
#define PRINTLN Serial.println
#else
#define PRINT
#define PRINTLN
#endif

// Support both the Uno and the Attiny85 boards as they will be some slight differences.

// Arduino Uno
#ifdef _AVR_IOM328P_H_
#define PORT_DIRECTION DDRD
#define PORT_OUTPUT PORTD
const uint8_t ENABLE_MASK = 0b10000000;
const auto SWITCH_BITS = 0b0111000;
const auto SET_PIN_SHIFT = 4;

// ATTINY85
#elif _AVR_IOTN85_H_
auto PORT_DIRECTION = DDRB;
auto PORT_OUTPUT = PORTB;
const uint8_t ENABLE_MASK = 0b10000000;
const auto SWITCH_BITS = 0b00111000;
const auto SET_PIN_SHIFT = 3;
/*
PB2 - SCL
PB0 - SDA
*/
#endif

class Fast4051
{
public:
  Fast4051()
  {
    begin();
  };

  void begin()
  {
    PORT_DIRECTION = ENABLE_MASK & SWITCH_BITS;
    PORT_OUTPUT = 0b00000000;
  }

  // Set the multiplexer pin to "pinToSet"
  void setPin(uint8_t pinToSet)
  {
    uint8_t out = pinToSet << SET_PIN_SHIFT;
    out |= (ENABLE_MASK & PORT_OUTPUT);
    //Write to the port in one go to avoid transistion errors
    PRINT("port: ");
    PRINTLN(out, BIN);
    PORT_OUTPUT = out;
    _currentPin = pinToSet;
  };
  // Get what pin is currently set
  int getCurrentPin()
  {
    return _currentPin;
  };
  uint8_t getPort()
  {
    return PORT_OUTPUT;
  };
  // Enable the 4051
  void on()
  {
    bitClear(PORT_OUTPUT, 1);
  };
  // Disable the 4051
  void off()
  {
    bitSet(PORT_OUTPUT, 1);
  };

protected:
  int _currentPin;

private:
};

enum class Side
{
  PurpleOne = 0,
  RedTwo = 1,
  GreenThree = 2,
  WhiteFour = 3,
  OrangeFive = 4,
  YellowSix = 5,
  None
};

// const float RANGE = 1.0;
ADXL345 accel;
Fast4051 mux;
Side lastSide = Side::None;

void setupTiltSensor()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
#ifdef UNO
  Wire.setClock(200000);
#endif
  // initialize device
  PRINTLN("Initializing I2C devices...");
  accel.initialize();
  accel.setAutoSleepEnabled(false);

  // verify connection
  PRINTLN("Testing device connections...");
  while (!accel.testConnection())
  {
    PRINTLN("ADXL345 connection failed. Trying again...");
    delay(1000);
  }

  accel.setRate(ADXL345_RATE_100);
  accel.setRange(ADXL345_RANGE_8G);
  PRINTLN("ADXL345 connection passed.");
}

int isOnSide(int a, int b, int c)
{
  const int ZERO_TRESHOLD = 40;
  const int G_THRESHOLD = 50;
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

  PRINT("mag: ");
  PRINTLN(mag);

  const int16_t MAGNITUDELIMIT = 80;
  if (mag < MAGNITUDELIMIT)
  {
    int xSide = isOnSide(ax, ay, az);
    int ySide = isOnSide(ay, ax, az);
    int zSide = isOnSide(az, ay, ax);

    if (xSide == 1 && ySide == 0 && zSide == 0)
      side = Side::PurpleOne;
    else if (xSide == -1 && ySide == 0 && zSide == 0)
      side = Side::RedTwo;
    else if (xSide == 0 && ySide == 1 && zSide == 0)
      side = Side::GreenThree;
    else if (xSide == 0 && ySide == -1 && zSide == 0)
      side = Side::WhiteFour;
    else if (xSide == 0 && ySide == 0 && zSide == 1)
      side = Side::OrangeFive;
    else if (xSide == 0 && ySide == 0 && zSide == -1)
      side = Side::YellowSix;
  }
  return side;
}

void setSwitch(Side side)
{
  uint8_t muxValue = mux.getCurrentPin();

  switch (side)
  {
  case Side::PurpleOne:
    muxValue = 4;
    break;
  case Side::RedTwo:
    muxValue = 6;
    break;
  case Side::GreenThree:
    muxValue = 7;
    break;
  case Side::WhiteFour:
    muxValue = 5;
    break;
  case Side::OrangeFive:
    muxValue = 2;
    break;
  case Side::YellowSix:
    muxValue = 1;
    break;
  default:
    break;
  }

  PRINT("mux: ");
  PRINTLN(muxValue);
  mux.setPin(muxValue);
}

void setup()
{
  // initialize serial communication
#ifdef DEBUG
  Serial.begin(38400);
#endif

#ifdef _AVR_IOM328P_H_
  PRINTLN("UNO");
#else
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(1, INPUT);
#endif
  mux.begin();
  // setupTiltSensor();
  PRINTLN("Ready player one");
}

void loop()
{
  // int16_t ax, ay, az;
  // accel.getAcceleration(&ax, &ay, &az);
  // // read raw accel measurements from device
  // // display tab-separated accel x/y/z values
  // PRINT("accel:\t");
  // PRINT(ax);
  // PRINT("\t");
  // PRINT(ay);
  // PRINT("\t");
  // PRINTLN(az);

  // Side side = calculateSide(ax, ay, az);

  // if (side != lastSide)
  // {
  //   PRINT("Side: ");
  //   PRINTLN(static_cast<int>(side));
  //   setSwitch(side);
  //   lastSide = side;
  // }
  for (int i = 0; i < 8; i++)
  {
    PRINT("mux: ");
    PRINTLN(i);
    mux.setPin(i);
    delay(1000);
  }
#ifdef DEBUG
  delay(500);
#endif
}