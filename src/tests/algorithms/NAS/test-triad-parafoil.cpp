/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <algorithms/NAS/StateInitializer.h>
#include <miosix.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorManager.h>
#include <sensors/calibration/SensorDataExtra/SensorDataExtra.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>

#include <cmath>
#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

void mpuInit();
void bmxCallback();

Vector3f nedMag = Vector3f(0.4747, 0.0276, 0.8797);

SPIBus spi1(SPI1);
MPU9250* mpu = nullptr;

int main()
{
    mpu = new MPU9250(spi1, sensors::mpu9250::cs::getPin());
    mpu->init();

    auto lastTick = getTime() / 1e6;
    while (true)
    {
        mpu->sample();
        auto data = mpu->getLastSample();

        Vector3f acceleration(data.accelerationX, data.accelerationY,
                              data.accelerationZ);

        Vector3f offset(15.9850903462129, -15.6775071377074, -33.8438469147423);
        Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                               data.magneticFieldZ);
        magneticField -= offset;
        magneticField = {magneticField[1], magneticField[0], -magneticField[2]};

        acceleration.normalize();
        magneticField.normalize();

        StateInitializer state;
        state.triad(acceleration, magneticField, nedMag);

        auto kalmanState = state.getInitX();
        if (!std::isnan(kalmanState(9)))
            printf("w%fwa%fab%fbc%fc\n", kalmanState(9), kalmanState(6),
                   kalmanState(7), kalmanState(8));

        Thread::nanoSleepUntil(lastTick + 20);
        lastTick = getTime() / 1e6;
    }
}
