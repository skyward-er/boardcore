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

#include <Common.h>
#include <drivers/HardwareTimer.h>
#include <drivers/interrupt/external_interrupts.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/BMX160/BMX160WithCorrection.h>
#include <sensors/calibration/SoftIronCalibration.h>

#include <fstream>
#include <iostream>

constexpr const char* CORRECTION_PARAMETER_FILE = "/sd/bmx160_params.csv";
constexpr const char* MAG_CALIBRATION_DATA_FILE =
    "/sd/bmx160_mag_calibration_data.csv";
constexpr int ACC_CALIBRATION_SAMPLES        = 200;
constexpr int ACC_CALIBRATION_SLEEP_TIME     = 25;  // [us]
constexpr int ACC_CALIBRATION_N_ORIENTATIONS = 6;

constexpr int MAG_CALIBRATION_SAMPLES  = 500;
constexpr int MAG_CALIBRATION_DURAITON = 60;  // [s]
constexpr int MAG_CALIBRATION_SLEEP_TIME =
    MAG_CALIBRATION_DURAITON * 1000 / MAG_CALIBRATION_SAMPLES;  // [us]

BMX160* bmx160;

/**
 * Orientations for accelerometer calibration
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
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    if (bmx160)
    {
        bmx160->IRQupdateTimestamp(TimestampTimer::getTimestamp());
    }
}

int menu();
bool askToContinue();
void waitForInput();

BMX160CorrectionParameters calibrateAccelerometer(
    BMX160CorrectionParameters correctionParameters);
BMX160CorrectionParameters calibrateMagnetometer(
    BMX160CorrectionParameters correctionParameters);
BMX160CorrectionParameters changeMinGyroCorrectionSamples(
    BMX160CorrectionParameters correctionParameters);

int main()
{
    TimestampTimer::enableTimestampTimer();

    // Enable interrupt from BMX pin
    enableExternalInterrupt(GPIOE_BASE, 5, InterruptTrigger::FALLING_EDGE);

    // Greet the user
    printf("\nWelcome to the calibration procedure!\n");

    // Read the current correciton parameters
    BMX160CorrectionParameters correctionParameters;
    correctionParameters =
        BMX160WithCorrection::readCorrectionParametersFromFile(
            CORRECTION_PARAMETER_FILE);

    // Make the user choose what to do
    switch (menu())
    {
        case 1:
            correctionParameters = calibrateAccelerometer(correctionParameters);
            break;
        case 2:
            correctionParameters = calibrateMagnetometer(correctionParameters);
            break;
        case 3:
            correctionParameters =
                changeMinGyroCorrectionSamples(correctionParameters);
            break;

        default:
            break;
    }

    // Save the correction parameters in the file
    {
        std::ofstream correctionParametersFile(CORRECTION_PARAMETER_FILE);
        correctionParametersFile << BMX160CorrectionParameters::header()
                                 << std::endl;
        correctionParameters.print(correctionParametersFile);
    }

    return 0;
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
    char choice;

    printf("Press [c] to continue, otherwise stop...");
    scanf("%s", &choice);

    return choice == 'c';
}

void waitForInput()
{
    printf("Press [c] to continue...");
    while (getchar() != 'c')
        ;
}

/**
 * The accelerometer calibration consists in finding the bias and coefficient
 * against target 9.8m/s^2 on the three axis positioning the sensor in all the 6
 * positions (+x, -x, +y, -y, +z, -z).
 */
BMX160CorrectionParameters calibrateAccelerometer(
    BMX160CorrectionParameters correctionParameters)
{
    Matrix<float, 3, 2> m;
    auto* calibrationModel =
        new SixParameterCalibration<AccelerometerData,
                                    ACC_CALIBRATION_SAMPLES *
                                        ACC_CALIBRATION_N_ORIENTATIONS>;

    SPIBus bus(SPI1);

    BMX160Config bmx_config;
    bmx_config.fifo_mode      = BMX160Config::FifoMode::HEADER;
    bmx_config.fifo_watermark = 100;
    bmx_config.fifo_int       = BMX160Config::FifoInterruptMode::PIN_INT1;

    bmx_config.temp_divider = 0;

    bmx_config.acc_range = BMX160Config::AccelerometerRange::G_16;
    bmx_config.gyr_range = BMX160Config::GyroscopeRange::DEG_2000;

    bmx_config.acc_odr = BMX160Config::OutputDataRate::HZ_1600;
    bmx_config.gyr_odr = BMX160Config::OutputDataRate::HZ_1600;
    bmx_config.mag_odr = BMX160Config::OutputDataRate::HZ_50;

    bmx_config.gyr_unit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(bus, miosix::sensors::bmx160::cs::getPin(), bmx_config);

    TRACE("Initializing BMX160...\n");

    if (!bmx160->init())
    {
        TRACE("Init failed! (code: %d)\n", bmx160->getLastError());
        return correctionParameters;
    }

    TRACE("Performing self-test...\n");

    if (!bmx160->selfTest())
    {
        TRACE("Self-test failed! (code: %d)\n", bmx160->getLastError());
        return correctionParameters;
    }

    TRACE("Initialization and self-test completed!\n");

    Vector3f avgAccel, vec;
    int fifoAccSampleCount;
    uint64_t accelTimestamp = 0;

    // Set reference vector, 1g on third axis
    calibrationModel->setReferenceVector({0, 0, 9.8});

    // Show the user the current correction values
    printf("\nCurrent bias vector\n");
    printf("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n",
           correctionParameters.accelParams(0, 1),
           correctionParameters.accelParams(1, 1),
           correctionParameters.accelParams(2, 1));
    printf("Matrix to be multiplied to the input vector\n");
    printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n",
           correctionParameters.accelParams(0, 0), 0.f, 0.f);
    printf("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f,
           correctionParameters.accelParams(1, 0), 0.f);
    printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f,
           correctionParameters.accelParams(2, 0));

    if (!askToContinue())
    {
        return correctionParameters;
    }

    printf(
        "Please note that the BMX axis, viewed facing the death stack x, "
        "are as follows:\n");
    printf("         z   x\n");
    printf("         ^   ^\n");
    printf("         |  /\n");
    printf("         | /\n");
    printf("  y <----/\n");

    for (unsigned i = 0; i < ACC_CALIBRATION_N_ORIENTATIONS; i++)
    {
        printf(
            "Step n.%d/%d, please rotate the death stack x so that the "
            "sensor "
            "is %s\n",
            i + 1, ACC_CALIBRATION_N_ORIENTATIONS,
            testHumanFriendlyDirection[i]);

        waitForInput();

        printf("Reding data and feeding the model...\n");

        // Sample the sensor and feed the data to the calibration model
        int count = 0;
        while (count < ACC_CALIBRATION_SAMPLES)
        {
            bmx160->sample();
            avgAccel = {0, 0, 0};

            // Read all the data contained in the fifo
            uint8_t size       = bmx160->getLastFifoSize();
            fifoAccSampleCount = 0;
            for (int i = 0; i < size && count < ACC_CALIBRATION_SAMPLES; i++)
            {
                BMX160Data fifoElement = bmx160->getFifoElement(i);

                // Read acceleration data
                if (fifoElement.accel_timestamp > accelTimestamp)
                {
                    static_cast<AccelerometerData>(fifoElement) >> vec;
                    avgAccel += vec;

                    accelTimestamp = fifoElement.accel_timestamp;
                    fifoAccSampleCount++;
                }
            }
            avgAccel /= fifoAccSampleCount;

            AccelerometerData data;
            data << avgAccel;

            miosix::Thread::sleep(ACC_CALIBRATION_SLEEP_TIME);

            calibrationModel->feed(data, orientations[i]);
            count++;
        }

        printf("Orientation calibration completed\n");
    }

    printf("Computing the result....");
    SixParameterCorrector<AccelerometerData> corrector =
        calibrationModel->computeResult();

    corrector >> m;
    corrector >> correctionParameters.accelParams;

    printf("Bias vector\n");
    printf("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", m(0, 1), m(1, 1),
           m(2, 1));
    printf("Matrix to be multiplied to the input vector\n");
    printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", m(0, 0), 0.f, 0.f);
    printf("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, m(1, 0), 0.f);
    printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f, m(2, 0));

    return correctionParameters;
}

BMX160CorrectionParameters calibrateMagnetometer(
    BMX160CorrectionParameters correctionParameters)
{
    Matrix<float, 3, 2> m;
    auto* calibrationModel = new SoftIronCalibration<MAG_CALIBRATION_SAMPLES>;
    Vector3f avgMag{0, 0, 0}, vec;

    SPIBus bus(SPI1);

    BMX160Config bmx_config;
    bmx_config.fifo_mode = BMX160Config::FifoMode::DISABLED;

    bmx_config.temp_divider = 0;

    bmx_config.acc_range = BMX160Config::AccelerometerRange::G_16;
    bmx_config.gyr_range = BMX160Config::GyroscopeRange::DEG_2000;

    bmx_config.acc_odr = BMX160Config::OutputDataRate::HZ_1600;
    bmx_config.gyr_odr = BMX160Config::OutputDataRate::HZ_1600;
    bmx_config.mag_odr = BMX160Config::OutputDataRate::HZ_50;

    bmx_config.gyr_unit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(bus, miosix::sensors::bmx160::cs::getPin(), bmx_config);

    TRACE("Initializing BMX160...\n");

    if (!bmx160->init())
    {
        TRACE("Init failed! (code: %d)\n", bmx160->getLastError());
        return correctionParameters;
    }

    TRACE("Performing self-test...\n");

    if (!bmx160->selfTest())
    {
        TRACE("Self-test failed! (code: %d)\n", bmx160->getLastError());
        return correctionParameters;
    }

    TRACE("Initialization and self-test completed!\n");

    // Show the user the current correction values
    printf("Current bias vector\n");
    printf("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n",
           correctionParameters.magnetoParams(0, 1),
           correctionParameters.magnetoParams(1, 1),
           correctionParameters.magnetoParams(2, 1));
    printf("Matrix to be multiplied to the input vector\n");
    printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n",
           correctionParameters.magnetoParams(0, 0), 0.f, 0.f);
    printf("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f,
           correctionParameters.magnetoParams(1, 0), 0.f);
    printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f,
           correctionParameters.magnetoParams(2, 0));

    printf("Now I will calibrate the magnetometer\n");
    printf(
        "Please, after starting the calibration, rotate the gyroscope in "
        "the "
        "most different directions.\n");
    printf("The calibration will run for %d seconds\n",
           MAG_CALIBRATION_DURAITON);

    if (!askToContinue())
    {
        return correctionParameters;
    }

    // Sample the sensor and feed the data to the calibration model
    int count = 0;
    while (count < MAG_CALIBRATION_SAMPLES)
    {
        bmx160->sample();

        BMX160Data data = bmx160->getLastSample();
        calibrationModel->feed(data);
        count++;

        miosix::Thread::sleep(MAG_CALIBRATION_SLEEP_TIME);
    }

    printf("Computing the result....");
    auto corrector = calibrationModel->computeResult();

    // Save the calibration data in the sd card
    // Save the correction parameters in the file
    {
        std::ofstream magnetometerCalibrationDataFile(
            MAG_CALIBRATION_DATA_FILE);

        auto samples        = calibrationModel->getSamples();
        int numberOfSamples = samples.rows();

        for (int i = 0; i < numberOfSamples; i++)
        {
            auto row = samples.row(i);
            for (int j = 0; j < row.cols(); j++)
            {
                magnetometerCalibrationDataFile << samples.row(i)(j) << ",";
            }
            magnetometerCalibrationDataFile << "\n";
        }
    }

    corrector >> m;
    corrector >> correctionParameters.magnetoParams;

    printf("b: the bias vector\n");
    printf("b = [    % 2.5f    % 2.5f    % 2.5f    ]\n\n", m(0, 1), m(1, 1),
           m(2, 1));
    printf("M: the matrix to be multiplied to the input vector\n");
    printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", m(0, 0), 0.f, 0.f);
    printf("M = |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, m(1, 0), 0.f);
    printf("    |    % 2.5f    % 2.5f    % 2.5f    |\n", 0.f, 0.f, m(2, 0));

    return correctionParameters;
}

BMX160CorrectionParameters changeMinGyroCorrectionSamples(
    BMX160CorrectionParameters correctionParameters)
{
    // Show the user the current parameter
    printf(
        "The current minimun number of gyroscope samples for calibration "
        "is "
        "%d\n",
        correctionParameters.minGyroSamplesForCalibration);

    if (askToContinue())
    {
        printf("Insert the new value: ");
        scanf("%d", &correctionParameters.minGyroSamplesForCalibration);
    }

    return correctionParameters;
}