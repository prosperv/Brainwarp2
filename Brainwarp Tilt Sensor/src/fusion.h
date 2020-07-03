#include <Arduino.h>
#include <Wire.h>
#include <L3G.h>
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

        if (!_gyro.init(L3G::device_4200D))
        {
            Serial.println("Failed to autodetect gyro type!");
            while (1)
                ;
        }
        _gyro.enableDefault();

        // DR = 01 (200 Hz ODR); BW = 11 (70 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
        _gyro.writeReg(L3G::CTRL_REG1, 0b01101111);
        // FS = 11 (2000dps)
        _gyro.writeReg(L3G::CTRL_REG4, 0b00110000);
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

    IMUSide staticAnalysis(double accelValue[], double gryoValue[])
    {
        IMUSide side = IMUSide::None;

        // If the gryo values are high the toy is spinning.
        const double GRYO_STATIC_THRESHOLD = 100;
        if (gryoValue[0] + gryoValue[1] + gryoValue[2] > GRYO_STATIC_THRESHOLD)
        {
            return IMUSide::None;
        }

        auto mag = sqrt(sq(accelValue[0]) + sq(accelValue[1]) + sq(accelValue[2]));

        const double MAG_UPPER_LIMIT = 1.3;
        const double MAG_LOWER_LIMIT = 0.5;
        if (mag > MAG_LOWER_LIMIT && mag < MAG_UPPER_LIMIT)
        {
            int xSide = isOnSide(accelValue[0], accelValue[1], accelValue[2]);
            int ySide = isOnSide(accelValue[1], accelValue[0], accelValue[2]);
            int zSide = isOnSide(accelValue[2], accelValue[1], accelValue[0]);

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

    void computeScaledGyro(double scaledGyro[3], L3G::vector<int16_t> g)
    {
        const double dpsScale = 2000;
        const double maxRange = 32768 - 1;
        const double extraCorrection = 1.23;
        const auto correctionFactor = dpsScale / maxRange * extraCorrection;

        scaledGyro[0] = static_cast<double>(g.x) * correctionFactor;
        scaledGyro[1] = static_cast<double>(g.y) * correctionFactor;
        scaledGyro[2] = static_cast<double>(g.z) * correctionFactor;
    };

    IMUSide process()
    {
        double scaledAccel[3];
        double scaledGyro[3];
        auto lastReadTime = micros();

        _accel.get_Gxyz(scaledAccel);
        _gyro.read();
        computeScaledGyro(scaledGyro, _gyro.g);

        /// Static: Toy is not moving and is stable
        IMUSide staticSide = staticAnalysis(scaledAccel, scaledGyro);

        return _lastValidSide;
    }

    L3G _gyro;
    ADXL345 _accel = ADXL345();
    IMUSide _lastValidSide = IMUSide::None;
    double _accumulatedDegree[3] = {0, 0, 0};
};