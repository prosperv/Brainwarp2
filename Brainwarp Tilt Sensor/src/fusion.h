#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_ADXL345.h> // SparkFun ADXL345 Library

enum class IMUSide
{
    None = 0,
    xPlus,
    xMinus,
    yPlus,
    yMinus,
    zPlus,
    zMinus,
};

class Fusion
{
public:
    void begin()
    {
        // join I2C bus (I2Cdev library doesn't do this automatically)
        Wire.begin();
        Wire.setClock(400000);
        // initialize device
        _accel.powerOn();
        _accel.setRangeSetting(8);
        _accel.setRate(100);
    };

    void stop(){};

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

    IMUSide calculateSide(double &ax, double &ay, double &az)
    {
        IMUSide side;

        auto mag = sqrt(sq(ax) + sq(ay) + sq(az));

        const double MAG_UPPER_LIMIT = 1.3;
        const double MAG_LOWER_LIMIT = 0.5;
        if (mag > MAG_LOWER_LIMIT && mag < MAG_UPPER_LIMIT)
        {
            int xSide = isOnSide(ax, ay, az);
            int ySide = isOnSide(ay, ax, az);
            int zSide = isOnSide(az, ay, ax);

            if (xSide == 1 && ySide == 0 && zSide == 0)
                side = IMUSide::xPlus;
            else if (xSide == -1 && ySide == 0 && zSide == 0)
                side = IMUSide::xMinus;
            else if (xSide == 0 && ySide == 1 && zSide == 0)
                side = IMUSide::yPlus;
            else if (xSide == 0 && ySide == -1 && zSide == 0)
                side = IMUSide::yMinus;
            else if (xSide == 0 && ySide == 0 && zSide == 1)
                side = IMUSide::zPlus;
            else if (xSide == 0 && ySide == 0 && zSide == -1)
                side = IMUSide::zMinus;
        }
        return side;
    };

    IMUSide process()
    {
        double scaledAccel[3];
        _accel.get_Gxyz(scaledAccel);

        _lastValidSide = calculateSide(scaledAccel[0], scaledAccel[1], scaledAccel[2]);

        return _lastValidSide;
    }

    ADXL345 _accel = ADXL345();
    IMUSide _lastValidSide = IMUSide::None;
};