#include <Arduino.h>
#include <Wire.h>
#include <L3G.h>
#include <SparkFun_ADXL345.h> // SparkFun ADXL345 Library

// #define DEBUG
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
static T myABS(T x) { return x > 0 ? x : -x; };

template <typename Ta, typename Tb>
static float vector_dot(const Ta *a, const Tb *b)
{
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
};

template <typename Ta, typename Tb, typename To>
static void vector_cross(To *out, const Ta *a, const Tb *b)
{
    out[0] = (a[1] * b[2]) - (a[2] * b[1]);
    out[1] = (a[2] * b[0]) - (a[0] * b[2]);
    out[2] = (a[0] * b[1]) - (a[1] * b[0]);
};

template <typename T>
static bool vector_equality(const T *left, const T *right)
{
    return left[0] == right[0] && left[1] == right[1] && left[2] == right[2];
};

template <typename T>
static void vector_set(T *left, const T *right)
{
    left[0] = right[0];
    left[1] = right[1];
    left[2] = right[2];
};

// Add vector and get unit vector.
static void vector_int_add(int out[], const int a[], const int b[])
{
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];

    if (out[0] != 0)
        out[0] /= myABS(out[0]);
    if (out[1] != 0)
        out[1] /= myABS(out[1]);
    if (out[2] != 0)
        out[2] /= myABS(out[2]);
};

template <typename T>
static T vector_abs_sum(const T x[])
{
    return myABS(x[0]) + myABS(x[1]) + myABS(x[2]);
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
        _gyro.writeReg(L3G::CTRL_REG1, 0b01111111);
        // FS = 11 (2000dps)
        _gyro.writeReg(L3G::CTRL_REG4, 0b00110000);
    };

    void stop()
    {
        auto byte = _gyro.readReg(L3G::CTRL_REG1);
        byte &= ~(0b00001000);
        _gyro.writeReg(L3G::CTRL_REG1, 0b01101111);
    };

    int isOnSide(const double sideToCheck, const double axis1, const double axis2)
    {
        // const double G_THRESHOLD = 0.60;
        const double G_DIFFERENCE_THRESHOLD = 0.3;

        // We know we're on a side if one axis has a high values while the other 2 axis are near zero.
        // Ex. x: 0.01, y: -0.01, z: 1.1
        // if (myABS(sideToCheck) > G_THRESHOLD && myABS(axis1) < ZERO_TRESHOLD && myABS(axis2) < ZERO_TRESHOLD)
        // if (myABS(sideToCheck) > G_THRESHOLD)
        if (myABS(sideToCheck) > myABS(axis1) + G_DIFFERENCE_THRESHOLD
            && myABS(sideToCheck) > myABS(axis2) + G_DIFFERENCE_THRESHOLD)
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

    /* 
    Static analysis uses the accelerometer to determine the orientation of the toy. However
    the accelerometer is only reliable when the toy is still and the more the toy move the less
    reliable the accelerometer is.
    */
    bool staticAnalysis(int vector[3], const double accelValue[3], const double gryoValue[3])
    {
        // Checking to see if the top is spinning too much.
        const double GRYO_STATIC_THRESHOLD = 200;
        const auto gyroFastMagnitude = myABS(gryoValue[0]) + myABS(gryoValue[1]) + myABS(gryoValue[2]);
        if (gyroFastMagnitude > GRYO_STATIC_THRESHOLD)
        {
            return false;
        }

        auto accelMagnitude = sqrt(vector_dot(accelValue, accelValue));

        // Even if the toy is not spinning, too much translational movement can lower our confidence in
        // determining orientation.
        const double MAG_UPPER_LIMIT = 1.4;
        const double MAG_LOWER_LIMIT = 0.6;
        if (accelMagnitude < MAG_LOWER_LIMIT && accelMagnitude > MAG_UPPER_LIMIT)
        {
            return false;
        }

        // Simplify the orientation vector to unit vectors.
        vector[0] = isOnSide(accelValue[0], accelValue[1], accelValue[2]);
        vector[1] = isOnSide(accelValue[1], accelValue[0], accelValue[2]);
        vector[2] = isOnSide(accelValue[2], accelValue[1], accelValue[0]);

        return true;
    };

    /*
    Dynamic analysis use the gryoscope to determine the change in orientation and by integrating
    we can get the new orientation. However over time the integration value will become 
    inaccurate. Also it cannot determine orientation on its own thus the use of the accelerometer. 
    */
    bool dynamicAnalysis(int newVector[3], const int lastVector[3], const unsigned long usTimeDelta, const double gryoValue[])
    {
        bool ret = false;
        double gryoValueCopy[] = {gryoValue[0],
                                  gryoValue[1],
                                  gryoValue[2]};

        // Need to know the current toy orientation else we exit.
        if (vector_dot(lastVector, lastVector) == 0)
            return;


        // Zero out gyro value that is parallel to gravity as that doesn't really
        // affect the orientation that we care about.
        if (lastVector[0] != 0)
            gryoValueCopy[0] = 0;
        if (lastVector[1] != 0)
            gryoValueCopy[1] = 0;
        if (lastVector[2] != 0)
            gryoValueCopy[2] = 0;

        // Integrate value of from the gryo
        const auto timeDelta = usTimeDelta / 1000000.0;
        _accumulatedDegree[0] += gryoValueCopy[0] * timeDelta;
        _accumulatedDegree[1] += gryoValueCopy[1] * timeDelta;
        _accumulatedDegree[2] += gryoValueCopy[2] * timeDelta;

        // Check if toy has rotated enough.
        int rotationVector[3] = {0, 0, 0};
        const double DEGREE_THRESHOLD = 60;
        if (myABS(_accumulatedDegree[0]) > DEGREE_THRESHOLD)
        {
            int value = _accumulatedDegree[0] > 0 ? 1 : -1;
            rotationVector[0] = value;
            ret = true;
        }
        else if (myABS(_accumulatedDegree[1]) > DEGREE_THRESHOLD)
        {
            int value = _accumulatedDegree[1] > 0 ? 1 : -1;
            rotationVector[1] = value;
            ret = true;
        }
        else if (myABS(_accumulatedDegree[2]) > DEGREE_THRESHOLD)
        {
            int value = _accumulatedDegree[2] > 0 ? 1 : -1;
            rotationVector[2] = value;
            ret = true;
        }
        if (ret)
        { 
            vector_cross(newVector, lastVector, rotationVector);
        }
        return ret;
    };

    // Convert raw data from gryo to degrees per second.
    void computeScaledGyro(double scaledGyro[3], const L3G::vector<int16_t> g)
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
        auto readTimeDelta = currentReadTime - _lastReadTime;
        _lastReadTime = currentReadTime;

        _accel.get_Gxyz(scaledAccel);
        _gyro.read();
        computeScaledGyro(scaledGyro, _gyro.g);

        int staticVector[3] = {0, 0, 0};
        bool staticStatus = staticAnalysis(staticVector, scaledAccel, scaledGyro);

        int dynamicVector[3] = {0, 0, 0};
        bool dynamicStatus = dynamicAnalysis(dynamicVector, _lastValidOrientation, readTimeDelta, scaledGyro);

        /// Static if true: Toy is not moving and is stable
        /// Dynamic if true: We rotated enough
        if (staticStatus || dynamicStatus)
        {
            resetAccumulatedDegree();
            vector_int_add(_currentOrientation, staticVector, dynamicVector);
        }


        // Save last valid orientation
        if (vector_abs_sum(_currentOrientation) == 1
            && !vector_equality(_lastValidOrientation, _currentOrientation))
        {
            // Check if the orientation somehow flipped
            if (
                myABS(_currentOrientation[0] - _lastValidOrientation[0]) > 1
                || myABS(_currentOrientation[1] - _lastValidOrientation[1]) > 1
                || myABS(_currentOrientation[2] - _lastValidOrientation[2]) > 1
            )
            {
                vector_set(_currentOrientation, _lastValidOrientation);
            }
            else
            {  
                vector_set(_lastValidOrientation, _currentOrientation);
            }
        }


#ifdef DEBUG
        // Process Time : 0.9ms
        // auto totalProcessTime = micros() - currentReadTime;
        // PRINT("Process Time: ");
        // PRINT(totalProcessTime); PRINT(" uSec");
        // PRINTLN();
#endif
        return _currentOrientation;
    }

    L3G _gyro;
    ADXL345 _accel = ADXL345();
    int _currentOrientation[3] = {1, 0, 0};
    int _lastValidOrientation[3] = {1, 0, 0};
    unsigned long _lastReadTime = micros();
    double _accumulatedDegree[3] = {0, 0, 0};
};
