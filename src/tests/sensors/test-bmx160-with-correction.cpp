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
constexpr int ACC_CALIBRATION_SAMPLES        = 200;
constexpr int ACC_CALIBRATION_SLEEP_TIME     = 25;  // [us]
constexpr int ACC_CALIBRATION_N_ORIENTATIONS = 6;

constexpr int MAG_CALIBRATION_DURATION           = 10;  // [s]
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
    {Direction::POSITIVE_Y, Direction::POSITIVE_Z},  // X up
    {Direction::POSITIVE_Z, Direction::POSITIVE_X},  // Y up
    {Direction::POSITIVE_Y, Direction::POSITIVE_X},  // Z down
    {Direction::POSITIVE_Z, Direction::POSITIVE_Y},  // X down
    {Direction::POSITIVE_X, Direction::POSITIVE_Z},  // Y down
};

constexpr const char* testHumanFriendlyDirection[]{
    "Z up", "X up", "Y up", "Z down", "X down", "Y down",
};

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI3_IRQHandlerImpl()
{
    if (bmx160)
        bmx160->IRQupdateTimestamp(TimestampTimer::getTimestamp());
}

int menu();
void waitForInput();

SixParametersCorrector calibrateMagnetometer(SixParametersCorrector oldCorr);
SixParametersCorrector calibrateGyroscope(SixParametersCorrector corr);

int main()
{
    // Enable interrupt from BMX pin
    enableExternalInterrupt(miosix::sensors::bmx160::intr::getPin().getPort(),
                            miosix::sensors::bmx160::intr::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    // Greet the user
    printf("\nWelcome to the calibration procedure!\n");

    // TODO: Read and then write the current correction parameters

    // Make the user choose what to do
    switch (menu())
    {
        case 1:
            calibrateGyroscope({});
            break;
        case 2:
            calibrateMagnetometer({});
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
    printf("1. Calibrate gyroscope\n");
    printf("2. Calibrate magnetometer\n");
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

SixParametersCorrector calibrateMagnetometer(SixParametersCorrector oldCorr)
{
    Matrix<float, 3, 2> m;
    auto* calibrationModel = new SoftAndHardIronCalibration;
    Vector3f avgMag{0, 0, 0}, vec;

    SPIBus bus(SPI4);

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

    TRACE("Initializing BMX160...\n");

    if (!bmx160->init())
    {
        TRACE("Init failed! (code: %d)\n", bmx160->getLastError());
        return oldCorr;
    }

    TRACE("Performing self-test...\n");

    if (!bmx160->selfTest())
    {
        TRACE("Self-test failed! (code: %d)\n", bmx160->getLastError());
        return oldCorr;
    }

    TRACE("Initialization and self-test completed!\n");

    // Show the user the current correction values
    printf("Current correction parameters:\n");
    printf("A = |% 2.5f    % 2.5f    % 2.5f|\n\n", oldCorr.getA()(0),
           oldCorr.getA()(1), oldCorr.getA()(2));
    printf("b = |% 2.5f    % 2.5f    % 2.5f|\n\n", oldCorr.getb()(0),
           oldCorr.getb()(1), oldCorr.getb()(2));

    printf("Now I will calibrate the magnetometer\n");
    printf(
        "Please, after starting the calibration, rotate the gyroscope in the "
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
                calibrationModel->feed(fifo.at(i));
            }
        },
        200);
    Logger::getInstance().start();
    scheduler.start();

    Thread::sleep(MAG_CALIBRATION_DURATION * 1000);

    scheduler.stop();
    Logger::getInstance().stop();

    printf("Computing the result....\n");
    auto newCorr = calibrationModel->computeResult();

    printf("New correction parameters:\n");
    printf("A = |% 2.5f    % 2.5f    % 2.5f|\n\n", newCorr.getA()(0),
           newCorr.getA()(1), newCorr.getA()(2));
    printf("b = |% 2.5f    % 2.5f    % 2.5f|\n\n", newCorr.getb()(0),
           newCorr.getb()(1), newCorr.getb()(2));

    return newCorr;
}

SixParametersCorrector calibrateGyroscope(SixParametersCorrector corr)
{
    Vector3f avgGyro{0, 0, 0};
    int count = 0;

    SPIBus bus(SPI4);

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

    TRACE("Initializing BMX160...\n");

    if (!bmx160->init())
    {
        TRACE("Init failed! (code: %d)\n", bmx160->getLastError());
        return corr;
    }

    TRACE("Performing self-test...\n");

    if (!bmx160->selfTest())
    {
        TRACE("Self-test failed! (code: %d)\n", bmx160->getLastError());
        return corr;
    }

    TRACE("Initialization and self-test completed!\n");

    printf("Current correction parameters:\n");
    printf("b = |% 2.5f    % 2.5f    % 2.5f|\n", corr.getb()(0), corr.getb()(1),
           corr.getb()(2));

    TRACE("Now starting the gyroscope calibration\n");

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

                Vector3f gyro = {data.angularSpeedX, data.angularSpeedY,
                                 data.angularSpeedZ};

                avgGyro = (avgGyro * count + gyro) / (count + 1);
                count++;
            }

            printf("b = |% 2.5f    % 2.5f    % 2.5f|\n", avgGyro(0), avgGyro(1),
                   avgGyro(2));
        },
        200);

    Logger::getInstance().start();
    scheduler.start();

    Thread::sleep(GYRO_CALIBRATION_DURATION * 1000);

    scheduler.stop();
    Logger::getInstance().stop();

    corr.setb(avgGyro);

    printf("New correction parameters:\n");
    printf("b = |% 2.5f    % 2.5f    % 2.5f|\n\n", corr.getb()(0),
           corr.getb()(1), corr.getb()(2));

    return corr;
}

/**
 * The accelerometer calibration consists in finding the bias and coefficient
 * against target 9.8m/s^2 on the three axis positioning the sensor in all the 6
 * positions (+x, -x, +y, -y, +z, -z).
 */
// BMX160CorrectionParameters calibrateAccelerometer(
//     BMX160CorrectionParameters correctionParameters)
// {
//     Matrix<float, 3, 2> m;
//     auto* calibrationModel =
//         new SixParameterCalibration<AccelerometerData,
//                                     ACC_CALIBRATION_SAMPLES *
//                                         ACC_CALIBRATION_N_ORIENTATIONS>;

//     SPIBus bus(SPI1);

//     BMX160Config bmxConfig;
//     bmxConfig.fifoMode      = BMX160Config::FifoMode::HEADER;
//     bmxConfig.fifoWatermark = 100;
//     bmxConfig.fifoInterrupt = BMX160Config::FifoInterruptPin::PIN_INT1;

//     bmxConfig.temperatureDivider = 0;

//     bmxConfig.accelerometerRange = BMX160Config::AccelerometerRange::G_16;
//     bmxConfig.gyroscopeRange     = BMX160Config::GyroscopeRange::DEG_2000;

//     bmxConfig.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_1600;
//     bmxConfig.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_1600;
//     bmxConfig.magnetometerRate      = BMX160Config::OutputDataRate::HZ_50;

//     bmxConfig.gyroscopeUnit = BMX160Config::GyroscopeMeasureUnit::RAD;

//     bmx160 = new BMX160(bus, miosix::sensors::bmx160::cs::getPin(),
//     bmxConfig);

//     TRACE("Initializing BMX160...\n");

//     if (!bmx160->init())
//     {
//         TRACE("Init failed! (code: %d)\n", bmx160->getLastError());
//         return correctionParameters;
//     }

//     TRACE("Performing self-test...\n");

//     if (!bmx160->selfTest())
//     {
//         TRACE("Self-test failed! (code: %d)\n", bmx160->getLastError());
//         return correctionParameters;
//     }

//     TRACE("Initialization and self-test completed!\n");

//     Vector3f avgAccel, vec;
//     int fifoAccSampleCount;
//     uint64_t accelTimestamp = 0;

//     // Set reference vector, 1g on third axis
//     calibrationModel->setReferenceVector({0, 0, 9.8});

//     // Show the user the current correction values
//     printf("\nCurrent bias vector\n");
//     printf("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n",
//            correctionParameters.accelParams(0, 1),
//            correctionParameters.accelParams(1, 1),
//            correctionParameters.accelParams(2, 1));
//     printf("Matrix to be multiplied to the input vector\n");
//     printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n",
//            correctionParameters.accelParams(0, 0), 0.f, 0.f);
//     printf("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f,
//            correctionParameters.accelParams(1, 0), 0.f);
//     printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f,
//            correctionParameters.accelParams(2, 0));

//     waitForInput();

//     printf(
//         "Please note that the BMX axis, viewed facing the death stack x, "
//         "are as follows:\n");
//     printf("         z   x\n");
//     printf("         ^   ^\n");
//     printf("         |  /\n");
//     printf("         | /\n");
//     printf("  y <----/\n");

//     for (unsigned i = 0; i < ACC_CALIBRATION_N_ORIENTATIONS; i++)
//     {
//         printf(
//             "Step n.%u/%d, please rotate the death stack x so that the "
//             "sensor "
//             "is %s\n",
//             i + 1, ACC_CALIBRATION_N_ORIENTATIONS,
//             testHumanFriendlyDirection[i]);

//         waitForInput();

//         printf("Reding data and feeding the model...\n");

//         // Sample the sensor and feed the data to the calibration model
//         int count = 0;
//         while (count < ACC_CALIBRATION_SAMPLES)
//         {
//             bmx160->sample();
//             avgAccel = {0, 0, 0};

//             // Read all the data contained in the fifo
//             uint8_t size       = bmx160->getLastFifoSize();
//             fifoAccSampleCount = 0;
//             for (int ii = 0; ii < size; ii++)
//             {
//                 BMX160Data fifoElement = bmx160->getFifoElement(ii);

//                 // Read acceleration data
//                 if (fifoElement.accelerationTimestamp > accelTimestamp)
//                 {
//                     static_cast<AccelerometerData>(fifoElement) >> vec;
//                     avgAccel += vec;

//                     accelTimestamp = fifoElement.accelerationTimestamp;
//                     fifoAccSampleCount++;
//                 }
//             }
//             avgAccel /= fifoAccSampleCount;

//             AccelerometerData data;
//             data << avgAccel;

//             miosix::Thread::sleep(ACC_CALIBRATION_SLEEP_TIME);

//             calibrationModel->feed(data, orientations[i]);
//             count++;
//         }

//         printf("Orientation calibration completed\n");
//     }

//     printf("Computing the result....");
//     SixParameterCorrector<AccelerometerData> corrector =
//         calibrationModel->computeResult();

//     corrector >> m;
//     corrector >> correctionParameters.accelParams;

//     printf("Bias vector\n");
//     printf("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", m(0, 1), m(1, 1),
//            m(2, 1));
//     printf("Matrix to be multiplied to the input vector\n");
//     printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", m(0, 0), 0.f, 0.f);
//     printf("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, m(1, 0), 0.f);
//     printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f, m(2, 0));

//     return correctionParameters;
// }
