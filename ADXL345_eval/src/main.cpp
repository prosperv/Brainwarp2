#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_ADXL345_U.h>

enum class Side
{
    Top,
    Bottom,
    Right,
    Left,
    Front,
    Back,
    None
};

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
Side lastSide = Side::None;
auto lastTime = micros();

int isOnSide(float a, float b, float c) {
    const float G_THRESHOLD = 8.0f;
    const float ZERO_TRESHOLD = 4.0f;
    if (abs(a) > G_THRESHOLD && abs(b) < ZERO_TRESHOLD && abs(c) <ZERO_TRESHOLD)
    {
        return a > 0 ? (1) : -1;
    }
    else {
        return 0;
    }
};

const int bufferSize = 50;
float magHistory[bufferSize];
int index = 0;
void recordMag(float mag)
{
  magHistory[index] = mag;
  index++;
  if (index >= bufferSize)
  {
    index = 0;
  }
}
void printMag()
{
  for(int i = 0; i < bufferSize; i++)
  {
    Serial.print(magHistory[index]);
    Serial.print(", ");
    index--;
    if (index < 0)
    {
      index = bufferSize;
    }
  }
    Serial.println();
}

void setup() {
  Serial.begin(115200);
  
  uint8_t regValue = accel.readRegister(ADXL345_REG_POWER_CTL);
  Serial.print("ADXL345_REG_POWER_CTL: ");
  Serial.println(regValue, BIN);
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);
  
  // accel.setDataRate(ADXL345_DATARATE_200_HZ);
  // accel.setDataRate(ADXL345_DATARATE_100_HZ);
  accel.setDataRate(ADXL345_DATARATE_50_HZ);

  Wire.setClock(100000);

  delay(1000);
  Serial.println("Starting loop.");
}

void loop() {
  delay(20);
  /* Get a new sensor event */ 
  static auto time1 = micros();
  static auto timeLastRead = time1;
  timeLastRead = time1;
  time1 = micros();

  sensors_event_t event; 
  accel.getEvent(&event);

  
  static auto time2 = micros();
  time2 = micros();
  int xSide = isOnSide(event.acceleration.x, event.acceleration.y, event.acceleration.z);
  int ySide = isOnSide(event.acceleration.y, event.acceleration.x, event.acceleration.z);
  int zSide = isOnSide(event.acceleration.z, event.acceleration.y, event.acceleration.x);

  Side currentSide = lastSide;

  static auto time3 = micros();
  time3 = micros();
  
  auto mag = sqrt(sq(event.acceleration.x) + sq(event.acceleration.y) + sq(event.acceleration.z)); 
  if (mag < 14)
  {
    if (xSide == 1 && ySide == 0 && zSide == 0)
      currentSide = Side::Top;
    else if (xSide == -1 && ySide == 0 && zSide == 0)
      currentSide = Side::Bottom;
    else if (xSide == 0 && ySide == 1 && zSide == 0)
      currentSide = Side::Front;
    else if (xSide == 0 && ySide == -1 && zSide == 0)
      currentSide = Side::Back;
    else if (xSide == 0 && ySide == 0 && zSide == 1)
      currentSide = Side::Left;
    else if (xSide == 0 && ySide == 0 && zSide == -1)
      currentSide = Side::Right;
  }
  recordMag(mag);


  if (lastSide != currentSide) 
  {
    static auto time4 = micros();
    time4 = micros();
    Serial.print(static_cast<int>(currentSide));
    Serial.println("");
    lastSide = currentSide;
    printMag();

  /* Display the results (acceleration is measured in m/s^2) */
    // Serial.print(event.acceleration.x); Serial.print(", ");
    // Serial.print(event.acceleration.y); Serial.print(", ");
    // Serial.print(event.acceleration.z); Serial.print(",  ");Serial.println("");

    Serial.print("mag: ");
    Serial.println(mag);
    Serial.print(time1-timeLastRead);  Serial.print(", ");
    Serial.print(time2-time1);  Serial.print(", ");
    Serial.print(time3-time2);  Serial.print(", ");
    Serial.print(time4-time3);  Serial.print(", ");
    Serial.println("");
  }
}
