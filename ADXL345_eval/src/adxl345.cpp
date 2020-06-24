#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_ADXL345.h> // SparkFun ADXL345 Library

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

ADXL345 accel = ADXL345(); // USE FOR I2C COMMUNICATION
Side lastSide = Side::None;
auto lastTime = micros();

int isOnSide(int a, int b, int c)
{
    const auto G_THRESHOLD = 50;
    const auto ZERO_TRESHOLD = 40;
    if (abs(a) > G_THRESHOLD && abs(b) < ZERO_TRESHOLD && abs(c) < ZERO_TRESHOLD)
    {
        return a > 0 ? (1) : -1;
    }
    else
    {
        return 0;
    }
};

const int bufferSize = 50;
int magHistory[bufferSize];
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
    for (int i = 0; i < bufferSize; i++)
    {
        index--;
        if (index < 0)
        {
            index = bufferSize;
        }
        Serial.print(magHistory[index]);
        Serial.print(", ");
    }
    Serial.println();
}

void printValues(int &ax, int &ay, int &az)
{
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(ay);
    Serial.print(", ");
    Serial.print(az);
    Serial.println("");
}

void adxl345_setup()
{
    Serial.begin(115200);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
#ifdef _AVR_IOM328P_H_
    Wire.setClock(200000);
#endif
    // initialize device
    Serial.println("Initializing I2C devices...");
    accel.powerOn();
    accel.setRangeSetting(8);
    accel.setRate(100);

    int ax = 0, ay = 0, az = 0;
    accel.readAccel(&ax, &ay, &az);
    printValues(ax, ay, az);

    Serial.println("Ready Player One");
}

void adxl345_loop()
{
    delay(20);
    /* Get a new sensor event */
    static auto time1 = micros();
    static auto timeLastRead = time1;
    timeLastRead = time1;
    time1 = micros();

    int ax, ay, az;
    accel.readAccel(&ax, &ay, &az);
    // printValues(ax, ay, az);

    static auto time2 = micros();
    time2 = micros();

    Side currentSide = lastSide;
    auto mag = sqrt(sq(ax) + sq(ay) + sq(az));
    // Serial.print("mag: ");
    // Serial.println(mag);
    static auto time3 = micros();
    time3 = micros();
    if (mag < 75)
    {
        int xSide = isOnSide(ax, ay, az);
        int ySide = isOnSide(ay, ax, az);
        int zSide = isOnSide(az, ay, ax);
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

        Serial.print("mag: ");
        Serial.println(mag);
        Serial.print(time1 - timeLastRead);
        Serial.print(", ");
        Serial.print(time2 - time1);
        Serial.print(", ");
        Serial.print(time3 - time2);
        Serial.print(", ");
        Serial.print(time4 - time3);
        Serial.print(", ");
        Serial.println("");
    }
}
