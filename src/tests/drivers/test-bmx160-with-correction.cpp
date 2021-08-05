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

constexpr const char *CORRECTION_PARAMETER_FILE = "/sd/bmx160_params.csv";

constexpr int UPDATE_RATE = 5;  // Hz
bool stopSamplingThread   = false;

BMX160 *bmx160 = nullptr;

// BMX160 Watermark interrupt
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    if (bmx160)
    {
        bmx160->IRQupdateTimestamp(TimestampTimer::getTimestamp());
    }
}

void bmx160Sample(void *args);

int main()
{
    TimestampTimer::enableTimestampTimer();

    // Enable interrupt from BMX pin
    enableExternalInterrupt(GPIOE_BASE, 5, InterruptTrigger::FALLING_EDGE);

    SPIBus bus(SPI1);

    BMX160Config bmx_config;
    bmx_config.fifo_mode      = BMX160Config::FifoMode::HEADER;
    bmx_config.fifo_watermark = 500 / 4;
    bmx_config.fifo_int       = BMX160Config::FifoInterruptPin::PIN_INT1;

    bmx_config.temp_divider = 0;

    bmx_config.acc_range = BMX160Config::AccelerometerRange::G_16;
    bmx_config.gyr_range = BMX160Config::GyroscopeRange::DEG_2000;

    bmx_config.acc_odr = BMX160Config::OutputDataRate::HZ_1600;
    bmx_config.gyr_odr = BMX160Config::OutputDataRate::HZ_1600;
    bmx_config.mag_odr = BMX160Config::OutputDataRate::HZ_50;

    bmx_config.gyr_unit = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(bus, miosix::sensors::bmx160::cs::getPin(), bmx_config);

    // Read the correction parameters
    BMX160CorrectionParameters correctionParameters =
        BMX160WithCorrection::readCorrectionParametersFromFile(
            CORRECTION_PARAMETER_FILE);

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

    BMX160WithCorrection bmx160WithCorrection(
        bmx160, correctionParameters,
        {Direction::NEGATIVE_Z, Direction::NEGATIVE_Y});

    TRACE("Initializing BMX160...\n");

    if (!bmx160->init())
    {
        TRACE("Init failed! (code: %d)\n", bmx160->getLastError());
        return -1;
    }

    TRACE("Performing self-test...\n");

    if (!bmx160->selfTest())
    {
        TRACE("Self-test failed! (code: %d)\n", bmx160->getLastError());
        return -1;
    }

    TRACE("Initialization and self-test completed!\n");

    // Calibrate the sensor
    miosix::Thread *samplingThread =
        miosix::Thread::create(bmx160Sample, 2048, miosix::MAIN_PRIORITY,
                               nullptr, miosix::Thread::JOINABLE);
    if (!bmx160WithCorrection.calibrate())
    {
        TRACE("Calibration failed!\n");
        return -1;
    }
    stopSamplingThread = true;
    samplingThread->join();
    TRACE("Calibration completed\n");

    while (1)
    {
        bmx160->sample();
        bmx160WithCorrection.sample();

        BMX160Data data = bmx160->getLastSample();
        BMX160WithCorrectionData correctedData =
            bmx160WithCorrection.getLastSample();

        printf("\n");

        printf("[Raw]       ");
        printf("Mag: %7.3f %7.3f %7.3f", data.mag_x, data.mag_y, data.mag_z);
        printf("\tGyr: %7.3f %7.3f %7.3f", data.gyro_x, data.gyro_y,
               data.gyro_z);
        printf("\tAcc: %7.3f %7.3f %7.3f\n", data.accel_x, data.accel_y,
               data.accel_z);

        printf("[Corrected] ");
        printf("Mag: %7.3f %7.3f %7.3f", correctedData.mag_x,
               correctedData.mag_y, correctedData.mag_z);
        printf("\tGyr: %7.3f %7.3f %7.3f", correctedData.gyro_x,
               correctedData.gyro_y, correctedData.gyro_z);
        printf("\tAcc: %7.3f %7.3f %7.3f\n", correctedData.accel_x,
               correctedData.accel_y, correctedData.accel_z);

        miosix::Thread::sleep(1000 / UPDATE_RATE);
    }

    return 0;
}

void bmx160Sample(void *args)
{
    UNUSED(args);
    
    while (!stopSamplingThread)
    {
        // Sample the bmx160
        bmx160->sample();
        if (bmx160->getLastError() != SensorErrors::NO_ERRORS)
        {
            TRACE("Failed to read data! (error: %d)\n", bmx160->getLastError());
            continue;
        }

        miosix::Thread::sleep(1000 / UPDATE_RATE);
    }
}