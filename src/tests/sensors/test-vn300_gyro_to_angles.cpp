/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Lorenzo Cucchi, Fabrizio Monti
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <drivers/timer/TimestampTimer.h>
#include <sensors/Vectornav/VN300/VN300.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

#include "interfaces-impl/hwmapping.h"

using namespace std;
using namespace Eigen;

using namespace miosix;
using namespace Boardcore;

struct LogAngles
{
    float roll, pitch, yaw;

    LogAngles() : roll{0}, pitch{0}, yaw(0) {}
    LogAngles(float Roll, float Pitch, float Yaw)
        : roll{Roll}, pitch{Pitch}, yaw(Yaw)
    {
    }

    void getAngles(float x, float y, float z)
    {
        this->roll  = x * (180 / M_PI);
        this->pitch = y * (180 / M_PI);
        this->yaw   = z * (180 / M_PI);
    }

    static std::string header() { return "Roll,Pitch,Yaw,quaternionZ\n"; }

    void print(std::ostream& os) const
    {
        os << roll << "," << pitch << "," << yaw << "\n";
    }
};

void computeEulerAngles(const Vector3d& gyro, double dt, Vector3d& angles)
{
    Vector3d body_rate = gyro;
    Vector3d ass       = angles;

    Matrix3d G;
    G << cos(ass(1)), 0, -cos(ass(0)) * sin(ass(1)), 0, 1, sin(ass(0)),
        sin(ass(1)), 0,
        cos(ass(0)) *
            cos(ass(1));  // rotation matrix angular_rate --> body_rate

    Vector3d angular_rate = G.transpose() * body_rate;

    Vector3d new_angles =
        angles + angular_rate * (dt / 10000000);  // integration
    angles = new_angles;

    // rad --> deg
}

int main()
{
    const double dt = 20000;  // [us]
    LogAngles angles_log;

    VN300Data sample;

    const int baud = 115200;
    USART usart(USART2, baud);
    VN300 sensor(usart, baud, VN300Defs::SampleOptions::FULL,
                 VN300::CRCOptions::CRC_ENABLE_8, std::chrono::seconds(2));

    // Let the sensor start up
    Thread::sleep(1000);

    printf("Initializing sensor\n");
    if (!sensor.init())
    {
        printf("Error initializing the sensor!\n");
        return 0;
    }

    printf("Running self-test\n");
    if (!sensor.selfTest())
    {
        printf("Unable to execute self-test\n");
        return 0;
    }

    uint64_t t0     = TimestampTimer::getTimestamp();
    Vector3d angles = {0, 0, 0};

    while (true)
    {
        uint64_t t1 = TimestampTimer::getTimestamp();

        if (t1 - t0 >= dt)
        {
            printf("Sampling\n");

            sensor.sample();
            sample = sensor.getLastSample();
            Logger::getInstance().log(sample);  // getting last sample & logs

            Vector3d gyro = {sample.angularSpeedX, sample.angularSpeedY,
                             sample.angularSpeedZ};

            computeEulerAngles(gyro, dt, angles);

            angles_log.getAngles(angles(0), angles(1), angles(2));
            printf("Angles: %f, %f, %f\n", angles_log.pitch, angles_log.roll,
                   angles_log.yaw);
            Boardcore::Logger::getInstance().log(angles_log);

            t0 = TimestampTimer::getTimestamp();
        }

        Thread::sleep(20);
    }

    return 0;
}

