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

        auto mag = sqrt(sq(accelValue[0]) + sq(accelValue[1]) + sq(accelValue[2]));

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

        _lastReadTime = currentReadTime;
        _lastValidSide[0] = staticVector[0];
        _lastValidSide[1] = staticVector[1];
        _lastValidSide[2] = staticVector[2];
        return _lastValidSide;
    }

    L3G _gyro;
    ADXL345 _accel = ADXL345();
    int _lastValidSide[3] = {0, 0, 0};
    unsigned long _lastReadTime = micros();
    double _accumulatedDegree[3] = {0, 0, 0};
};