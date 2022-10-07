/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Riccardo Musso, Alberto Nidasio
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

#include <drivers/interrupt/external_interrupts.h>
#include <drivers/timer/TimestampTimer.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160WithCorrection.h>
#include <sensors/calibration/AxisOrientation.h>
#include <sensors/calibration/BiasCalibration/BiasCalibration.h>
#include <sensors/calibration/SixParameterCalibration/SixParameterCalibration.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <utils/Debug.h>

#include <fstream>
#include <iostream>

using namespace std;
using namespace miosix;
using namespace Eigen;
using namespace Boardcore;

constexpr const char* CORRECTION_PARAMETER_FILE = "/sd/bmx160_params.csv";
constexpr const char* MAG_CALIBRATION_DATA_FILE =
    "/sd/bmx160_mag_calibration_data.csv";

constexpr int ACC_CALIBRATION_SLEEP_TIME     = 10;  // [s]
constexpr int ACC_CALIBRATION_N_ORIENTATIONS = 6;

constexpr int MAG_CALIBRATION_DURATION           = 30;  // [s]
constexpr uint32_t MAG_CALIBRATION_SAMPLE_PERIOD = 20;  // [ms]

constexpr int GYRO_CALIBRATION_DURATION = 10;  // [s]

BMX160* bmx160;

/**
 * @brief Orientations for accelerometer calibration.
 *
 * The BMX160 reference frame view facing the death stack x is:
 *          z   x
 *          ^   ^
 *          |  /
 *          | /
 *   y <----/
 *
 * Each AxisOrthoOrientation values indicates how to change x and y
 */
AxisOrthoOrientation orientations[ACC_CALIBRATION_N_ORIENTATIONS]{
    {Direction::POSITIVE_X, Direction::POSITIVE_Y},  // Z up
    {Direction::POSITIVE_Z, Direction::POSITIVE_Y},  // X up
    {Direction::POSITIVE_X, Direction::POSITIVE_Z},  // Y up
    {Direction::POSITIVE_X, Direction::NEGATIVE_Y},  // Z down
    {Direction::NEGATIVE_Z, Direction::POSITIVE_Y},  // X down
    {Direction::POSITIVE_X, Direction::NEGATIVE_Z},  // Y down
};

constexpr const char* testHumanFriendlyDirection[]{
    "Z up", "X up", "Y up", "Z down", "X down", "Y down",
};

#if defined(_BOARD_STM32F429ZI_SKYWARD_DEATHST_X)
SPIBus bus(SPI1);

void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
#elif defined(_BOARD_STM32F429ZI_SKYWARD_DEATHST_V3)
SPIBus bus(SPI4);

void __attribute__((used)) EXTI3_IRQHandlerImpl()
{
#else
#error "Board not supported"
#endif
    if (bmx160)
        bmx160->IRQupdateTimestamp(TimestampTimer::getTimestamp());
}

int menu();
void waitForInput();

void calibrateAccelerometer();
void calibrateMagnetometer();
void calibrateGyroscope();

int main()
{
// Enable interrupt from BMX pin
#if defined(_BOARD_STM32F429ZI_SKYWARD_DEATHST_X)
    enableExternalInterrupt(miosix::sensors::bmx160::intr::getPin().getPort(),
                            miosix::sensors::bmx160::intr::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);
#elif defined(_BOARD_STM32F429ZI_SKYWARD_DEATHST_V3)
    enableExternalInterrupt(miosix::sensors::bmx160::intr::getPin().getPort(),
                            miosix::sensors::bmx160::intr::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);
#else
#error "Board not supported"
#endif

    // Greet the user
    printf("\nWelcome to the calibration procedure!\n");

    // Make the user choose what to do
    switch (menu())
    {
        case 1:
            calibrateAccelerometer();
            break;
        case 2:
            calibrateMagnetometer();
            break;
        case 3:
            calibrateGyroscope();
            break;

        default:
            break;
    }

    return 0;
}

int menu()
{
    int choice;

    printf("\nWhat do you want to do?\n");
    printf("1. Calibrate accelerometer\n");
    printf("2. Calibrate magnetometer\n");
    printf("3. Calibrate gyroscope\n");
    printf("\n>> ");
    scanf("%d", &choice);

    return choice;
}

void waitForInput()
{
    string temp;
    do
    {
        cout << "Write 'c' to continue:\n";
        getline(cin, temp);
    } while (temp != "c");
}

void calibrateAccelerometer()
{
    SixParametersCorrector correction;
    correction.fromFile("/sd/bmx160_accelerometer_correction.csv");

    SixParameterCalibration calibrationModel({0, 0, 9.8});

    BMX160Config bmxConfig;
    bmxConfig.fifoMode      = BMX160Config::FifoMode::HEADER;
    bmxConfig.fifoWatermark = 20;
    bmxConfig.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    bmxConfig.temperatureDivider = 0;

    bmxConfig.accelerometerRange = BMX160Config::AccelerometerRange::G_16;
    bmxConfig.gyroscopeRange     = BMX160Config::GyroscopeRange::DEG_2000;

    bmxConfig.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.magnetometerRate      = BMX160Config::OutputDataRate::HZ_100;

    bmxConfig.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(bus, miosix::sensors::bmx160::cs::getPin(), bmxConfig);

    printf("Initializing BMX160...\n");

    if (!bmx160->init())
        printf("Init failed! (code: %d)\n", bmx160->getLastError());

    printf("Performing self-test...\n");

    if (!bmx160->selfTest())
        printf("Self-test failed! (code: %d)\n", bmx160->getLastError());

    printf("Initialization and self-test completed!\n");

    // Show the user the current correction values
    printf("Current correction parameters:\n");
    printf("A = |% 2.5f    % 2.5f    % 2.5f|\n\n", correction.getA()(0),
           correction.getA()(1), correction.getA()(2));
    printf("b = |% 2.5f    % 2.5f    % 2.5f|\n\n", correction.getb()(0),
           correction.getb()(1), correction.getb()(2));

    waitForInput();

    printf(
        "Please note that the BMX axis, viewed facing the ELC bay, are as "
        "follows:\n");
    printf("         z   x\n");
    printf("         ^   ^\n");
    printf("         |  /\n");
    printf("         | /\n");
    printf("  y <----/\n");

    for (unsigned i = 0; i < ACC_CALIBRATION_N_ORIENTATIONS; i++)
    {
        printf(
            "Step n.%u/%d, please rotate the death stack x so that the "
            "sensor "
            "is %s\n",
            i + 1, ACC_CALIBRATION_N_ORIENTATIONS,
            testHumanFriendlyDirection[i]);

        waitForInput();

        printf("Reding data and feeding the model...\n");

        TaskScheduler scheduler;
        scheduler.addTask(
            [&]()
            {
                bmx160->sample();

                uint8_t fifoSize = bmx160->getLastFifoSize();
                auto& fifo       = bmx160->getLastFifo();

                for (uint8_t ii = 0; ii < fifoSize; ii++)
                {
                    Logger::getInstance().log(fifo.at(ii));
                    calibrationModel.feed(
                        static_cast<AccelerometerData>(fifo.at(ii)),
                        orientations[i]);
                }
            },
            200);
        Logger::getInstance().start();
        scheduler.start();

        Thread::sleep(ACC_CALIBRATION_SLEEP_TIME * 1000);

        scheduler.stop();
        Logger::getInstance().stop();
    }

    printf("Computing the result....\n");
    auto newCorrector = calibrationModel.computeResult();

    printf("New correction parameters:\n");
    printf("A = |% 2.5f    % 2.5f    % 2.5f|\n\n", newCorrector.getA()(0),
           newCorrector.getA()(1), newCorrector.getA()(2));
    printf("b = |% 2.5f    % 2.5f    % 2.5f|\n\n", newCorrector.getb()(0),
           newCorrector.getb()(1), newCorrector.getb()(2));

    newCorrector.toFile("/sd/bmx160_accelerometer_correction.csv");
}

void calibrateMagnetometer()
{
    SixParametersCorrector corrector;
    corrector.fromFile("/sd/bmx160_magnetometer_correction.csv");

    SoftAndHardIronCalibration calibrationModel;
    Vector3f avgMag{0, 0, 0}, vec;

    BMX160Config bmxConfig;
    bmxConfig.fifoMode      = BMX160Config::FifoMode::HEADER;
    bmxConfig.fifoWatermark = 20;
    bmxConfig.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    bmxConfig.temperatureDivider = 0;

    bmxConfig.accelerometerRange = BMX160Config::AccelerometerRange::G_16;
    bmxConfig.gyroscopeRange     = BMX160Config::GyroscopeRange::DEG_2000;

    bmxConfig.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.magnetometerRate      = BMX160Config::OutputDataRate::HZ_100;

    bmxConfig.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(bus, miosix::sensors::bmx160::cs::getPin(), bmxConfig);

    printf("Initializing BMX160...\n");

    if (!bmx160->init())
        printf("Init failed! (code: %d)\n", bmx160->getLastError());

    printf("Performing self-test...\n");

    if (!bmx160->selfTest())
        printf("Self-test failed! (code: %d)\n", bmx160->getLastError());

    printf("Initialization and self-test completed!\n");

    // Show the user the current correction values
    printf("Current correction parameters:\n");
    printf("A = |% 2.5f    % 2.5f    % 2.5f|\n\n", corrector.getA()(0),
           corrector.getA()(1), corrector.getA()(2));
    printf("b = |% 2.5f    % 2.5f    % 2.5f|\n\n", corrector.getb()(0),
           corrector.getb()(1), corrector.getb()(2));

    printf("Now I will calibrate the magnetometer\n");
    printf(
        "Please, after starting the calibration, rotate the gyroscope in "
        "the "
        "most different directions.\n");
    printf("The calibration will run for %d seconds\n",
           MAG_CALIBRATION_DURATION);

    waitForInput();

    printf("Calibration started, rotate the stack!\n");

    TaskScheduler scheduler;
    scheduler.addTask(
        [&]()
        {
            bmx160->sample();

            uint8_t fifoSize = bmx160->getLastFifoSize();
            auto& fifo       = bmx160->getLastFifo();

            for (uint8_t i = 0; i < fifoSize; i++)
            {
                Logger::getInstance().log(fifo.at(i));
                calibrationModel.feed(fifo.at(i));
            }
        },
        200);
    Logger::getInstance().start();
    scheduler.start();

    Thread::sleep(MAG_CALIBRATION_DURATION * 1000);

    scheduler.stop();
    Logger::getInstance().stop();

    printf("Computing the result....\n");
    auto newCorrector = calibrationModel.computeResult();

    printf("New correction parameters:\n");
    printf("A = |% 2.5f    % 2.5f    % 2.5f|\n\n", newCorrector.getA()(0),
           newCorrector.getA()(1), newCorrector.getA()(2));
    printf("b = |% 2.5f    % 2.5f    % 2.5f|\n\n", newCorrector.getb()(0),
           newCorrector.getb()(1), newCorrector.getb()(2));

    newCorrector.toFile("/sd/bmx160_magnetometer_correction.csv");
}

void calibrateGyroscope()
{
    BiasCalibration calibrationModel;
    int count = 0;

    BMX160Config bmxConfig;
    bmxConfig.fifoMode      = BMX160Config::FifoMode::HEADER;
    bmxConfig.fifoWatermark = 20;
    bmxConfig.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

    bmxConfig.temperatureDivider = 0;

    bmxConfig.accelerometerRange = BMX160Config::AccelerometerRange::G_16;
    bmxConfig.gyroscopeRange     = BMX160Config::GyroscopeRange::DEG_2000;

    bmxConfig.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_100;
    bmxConfig.magnetometerRate      = BMX160Config::OutputDataRate::HZ_100;

    bmxConfig.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(bus, miosix::sensors::bmx160::cs::getPin(), bmxConfig);

    printf("Initializing BMX160...\n");

    if (!bmx160->init())
        printf("Init failed! (code: %d)\n", bmx160->getLastError());

    printf("Performing self-test...\n");

    if (!bmx160->selfTest())
        printf("Self-test failed! (code: %d)\n", bmx160->getLastError());

    printf("Initialization and self-test completed!\n");

    printf(
        "Now starting the gyroscope calibration, leave the stack perfectly "
        "still\n");

    TaskScheduler scheduler;
    scheduler.addTask(
        [&]()
        {
            bmx160->sample();

            uint8_t fifoSize = bmx160->getLastFifoSize();
            auto& fifo       = bmx160->getLastFifo();

            for (uint8_t i = 0; i < fifoSize; i++)
            {
                auto data = fifo.at(i);
                Logger::getInstance().log(data);

                calibrationModel.feed({data.angularVelocityX,
                                       data.angularVelocityY,
                                       data.angularVelocityZ});
                count++;
            }
        },
        200);

    Logger::getInstance().start();
    scheduler.start();

    Thread::sleep(GYRO_CALIBRATION_DURATION * 1000);

    scheduler.stop();
    Logger::getInstance().stop();

    auto corrector = calibrationModel.computeResult();

    printf("New correction parameters:\n");
    printf("b = |% 2.5f    % 2.5f    % 2.5f|\n\n", corrector.getb()(0),
           corrector.getb()(1), corrector.getb()(2));
}
