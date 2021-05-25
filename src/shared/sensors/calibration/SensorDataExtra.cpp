#include "SensorDataExtra.h"

using namespace Eigen;

void operator<<(AccelerometerData& lhs, const Vector3f& rhs)
{
    lhs.accel_x = rhs[0];
    lhs.accel_y = rhs[1];
    lhs.accel_z = rhs[2];
}

void operator<<(GyroscopeData& lhs, const Vector3f& rhs)
{
    lhs.gyro_x = rhs[0];
    lhs.gyro_y = rhs[1];
    lhs.gyro_z = rhs[2];
}

void operator<<(MagnetometerData& lhs, const Vector3f& rhs)
{
    lhs.mag_x = rhs[0];
    lhs.mag_y = rhs[1];
    lhs.mag_z = rhs[2];
}

void operator>>(const AccelerometerData& lhs, Vector3f& rhs)
{
    rhs[0] = lhs.accel_x;
    rhs[1] = lhs.accel_y;
    rhs[2] = lhs.accel_z;
}

void operator>>(const GyroscopeData& lhs, Vector3f& rhs)
{
    rhs[0] = lhs.gyro_x;
    rhs[1] = lhs.gyro_y;
    rhs[2] = lhs.gyro_z;
}

void operator>>(const MagnetometerData& lhs, Vector3f& rhs)
{
    rhs[0] = lhs.mag_x;
    rhs[1] = lhs.mag_y;
    rhs[2] = lhs.mag_z;
}


