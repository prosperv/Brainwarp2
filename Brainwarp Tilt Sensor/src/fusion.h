#include <Arduino.h>
#include <Wire.h>
#include <L3G.h>
#include <SparkFun_ADXL345.h> // SparkFun ADXL345 Library

#define DEBUG
#include "debug.h"

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

// The normal abs for some reason don't work well with float and double
template <typename T>
T myABS(T x) { return x > 0 ? x : -x; };

template <typename Ta, typename Tb>
float vector_dot(const Ta *a, const Tb *b)
{
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
};

void vector_int_add(int out[], const int a[], const int b[])
{
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];

    if (out[0] != 0)
        out[0] /= out[0];
    if (out[1] != 0)
        out[1] /= out[1];
    if (out[2] != 0)
        out[2] /= out[2];
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
        _accel.setRangeSetting(2);
        _accel.setRate(100);

        if (!_gyro.init(L3G::device_4200D))
        {
            PRINTLN("Failed to autodetect gyro type!");
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

    int isOnSide(double sideToCheck, double axis1, double axis2)
    {
        const double ZERO_TRESHOLD = 0.58;
        const double G_THRESHOLD = 0.75;

        // We know we're on a side if one axis has a high values while the other 2 axsi are near zero.
        // Ex. x: 0.01, y: -0.01, z: 1.1
        if (myABS(sideToCheck) > G_THRESHOLD && myABS(axis1) < ZERO_TRESHOLD && myABS(axis2) < ZERO_TRESHOLD)
        {
            return sideToCheck > 0 ? (1) : -1;
        }
        else
        {
            return 0;
        }
    };

    void resetAccumulatedDegree()
    {
        _accumulatedDegree[0] = 0;
        _accumulatedDegree[1] = 0;
        _accumulatedDegree[2] = 0;
    }

    void staticAnalysis(int vector[3], const double accelValue[3], const double gryoValue[3])
    {
        // If the gryo values are high the toy is spinning.
        const double GRYO_STATIC_THRESHOLD = 100;
        if (gryoValue[0] + gryoValue[1] + gryoValue[2] > GRYO_STATIC_THRESHOLD)
        {
            return;
        }

        auto mag = sqrt(vector_dot(accelValue, accelValue));

        const double MAG_UPPER_LIMIT = 1.3;
        const double MAG_LOWER_LIMIT = 0.5;
        if (mag > MAG_LOWER_LIMIT && mag < MAG_UPPER_LIMIT)
        {
            vector[0] = isOnSide(accelValue[0], accelValue[1], accelValue[2]);
            vector[1] = isOnSide(accelValue[1], accelValue[0], accelValue[2]);
            vector[2] = isOnSide(accelValue[2], accelValue[1], accelValue[0]);
        }
    };

    IMUSide dynamicAnalysis(IMUSide lastSide, unsigned long usTimeDelta, double gryoValue[])
    {
        double gryoValueCopy[] = {gryoValue[0],
                                  gryoValue[1],
                                  gryoValue[2]};

        // Zero out the axis if the toy is already on that axis, since the toy does not make
        // moves to the opposite ends, and we don't care about landing on the same side, since
        // the static analysis will handle that.
        switch (lastSide)
        {
        case IMUSide::xMinus:
        case IMUSide::xPlus:
            gryoValueCopy[0] = 0;
            break;
        case IMUSide::yMinus:
        case IMUSide::yPlus:
            gryoValueCopy[1] = 0;
            break;
        case IMUSide::zMinus:
        case IMUSide::zPlus:
            gryoValueCopy[2] = 0;
            break;
        default:
            return IMUSide::None;
            break;
        }

        const auto timeDelta = usTimeDelta / 1000000.0;
        _accumulatedDegree[0] = gryoValueCopy[0] * timeDelta;
        _accumulatedDegree[1] = gryoValueCopy[1] * timeDelta;
        _accumulatedDegree[2] = gryoValueCopy[2] * timeDelta;

        // const double DEGREE_THRESHOLD = 30;
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

    int *process()
    {
        double scaledAccel[3];
        double scaledGyro[3];
        auto currentReadTime = micros();
        auto readTimeDelta = _lastReadTime - currentReadTime;

        _accel.get_Gxyz(scaledAccel);
        _gyro.read();
        computeScaledGyro(scaledGyro, _gyro.g);

        /// Static: Toy is not moving and is stable
        int staticVector[3] = {0, 0, 0};
        staticAnalysis(staticVector, scaledAccel, scaledGyro);
        int dynamicVector[3] = {0, 0, 0};
        // dynamicAnalysis(dynamicVector, _lastValidSide, readTimeDelta, scaledGyro);

        vector_int_add(_lastValidSide, staticVector, dynamicVector);
        _lastReadTime = currentReadTime;

        auto totalProcessTime = currentReadTime - micros();
        return _lastValidSide;
    }

    L3G _gyro;
    ADXL345 _accel = ADXL345();
    int _lastValidSide[3] = {0, 0, 0};
    unsigned long _lastReadTime = micros();
    double _accumulatedDegree[3] = {0, 0, 0};
};