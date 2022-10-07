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

#include <sensors/MPU9250/MPU9250.h>
#include <sensors/SensorManager.h>
#include <sensors/calibration/AxisOrientation.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <utils/Debug.h>

#include <iostream>

using namespace Boardcore;
using namespace Eigen;
using namespace miosix;

int menu();
bool askToContinue();

void calibrateMagnetometer();

constexpr int MAG_CALIBRATION_DURATION = 30;  // [s]
constexpr int IMU_SAMPLE_PERIOD        = 10;  // [ms] 100Hz

int main()
{
    // Greet the user
    printf("\nWelcome to the calibration procedure!\n");

    // Make the user choose what to do
    switch (menu())
    {
        case 2:
            calibrateMagnetometer();
            break;

        default:
            break;
    }

    printf("end\n");
    reboot();
}

int menu()
{
    int choice;

    printf("\nWhat do you want to do?\n");
    printf("1. Calibrate accelerometer\n");
    printf("2. Calibrate magnetometer\n");
    printf("3. Set minimum gyroscope samples for calibration\n");
    printf("\n>> ");
    scanf("%d", &choice);

    return choice;
}

bool askToContinue()
{
    char c;

    std::cout << "Write 'c' to continue, otherwise stop:\n";
    scanf("%c", &c);

    return c != 'c';
}

void calibrateMagnetometer()
{
    SPIBus spiBus(SPI1);
    MPU9250 mpu(spiBus, sensors::mpu9250::cs::getPin());
    mpu.init();

    printf("Now the magnetometer calibration will begin\n");
    printf("Please, rotate the gyroscope A LOT!\n");
    printf("The calibration will run for %d seconds\n",
           MAG_CALIBRATION_DURATION);

    // if (!askToContinue())
    //     return;
    printf("Starting...\n");

    // Prepare the calibration model
    SoftAndHardIronCalibration calibration;

    // Prepare and start the sensor manager
    TaskScheduler scheduler;
    scheduler.addTask([]() { printf("...\n"); }, 500);
    scheduler.start();

    // Wait and then stop the sampling
    auto startTick = getTick();
    auto lastTick  = startTick;
    while (getTick() - startTick < MAG_CALIBRATION_DURATION * 1e3)
    {
        mpu.sample();
        calibration.feed(mpu.getLastSample());
        Thread::sleepUntil(lastTick + IMU_SAMPLE_PERIOD);
        lastTick = getTick();
    }

    scheduler.stop();

    printf("Computing the result...\n");

    auto correction = calibration.computeResult();

    printf("b: the bias vector\n");
    std::cout << correction.getb().transpose() << std::endl;
    printf("A: the gain to be multiplied to the input vector\n");
    std::cout << correction.getA().transpose() << std::endl;
}
