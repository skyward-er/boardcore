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

#include <algorithms/NAS/NAS.h>
#include <algorithms/NAS/StateInitializer.h>
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/SensorManager.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

NASConfig getEKConfig();
void bmxInit();
void bmxCallback();

constexpr uint64_t CALIBRATION_TIMEOUT = 5 * 1e6;

Vector3f nedMag = Vector3f(0.47472049, 0.02757190, 0.87970463);

StateInitializer* stateInitializer;
NAS* nas;

SPIBus spi1(SPI1);
BMX160* bmx160 = nullptr;

Boardcore::SensorManager::SensorMap_t sensorsMap;
Boardcore::SensorManager* sensorManager = nullptr;

int main()
{
    bmxInit();

    sensorManager    = new SensorManager(sensorsMap);
    stateInitializer = new StateInitializer();
    nas              = new NAS(getEKConfig());

    // Logger::getInstance().start();
    TimestampTimer::resetTimestamp();
    sensorManager->start();

    while (true)
        Thread::sleep(1000);
}

NASConfig getEKConfig()
{
    NASConfig config;

    config.T              = 0.02f;
    config.SIGMA_BETA     = 0.0001f;
    config.SIGMA_W        = 0.3f;
    config.SIGMA_ACC      = 0.1f;
    config.SIGMA_MAG      = 0.1f;
    config.SIGMA_GPS      = 10.0f;
    config.SIGMA_BAR      = 4.3f;
    config.SIGMA_POS      = 10.0;
    config.SIGMA_VEL      = 10.0;
    config.P_POS          = 1.0f;
    config.P_POS_VERTICAL = 10.0f;
    config.P_VEL          = 1.0f;
    config.P_VEL_VERTICAL = 10.0f;
    config.P_ATT          = 0.01f;
    config.P_BIAS         = 0.01f;
    config.SATS_NUM       = 6.0f;

    // Normalized magnetic field in Milan
    config.NED_MAG = nedMag;

    return config;
}

void bmxInit()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_8;

    BMX160Config bmx_config;
    bmx_config.fifoMode              = BMX160Config::FifoMode::HEADER;
    bmx_config.fifoWatermark         = 80;
    bmx_config.fifoInterrupt         = BMX160Config::FifoInterruptPin::PIN_INT1;
    bmx_config.temperatureDivider    = 1;
    bmx_config.accelerometerRange    = BMX160Config::AccelerometerRange::G_16;
    bmx_config.gyroscopeRange        = BMX160Config::GyroscopeRange::DEG_1000;
    bmx_config.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_100;
    bmx_config.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_100;
    bmx_config.magnetometerRate      = BMX160Config::OutputDataRate::HZ_100;
    bmx_config.gyroscopeUnit         = BMX160Config::GyroscopeMeasureUnit::RAD;

    bmx160 = new BMX160(spi1, miosix::sensors::bmx160::cs::getPin(), bmx_config,
                        spiConfig);

    SensorInfo info("BMX160", 20, &bmxCallback);

    sensorsMap.emplace(std::make_pair(bmx160, info));
}

void bmxCallback()
{
    static int meanCount    = 0;
    static bool calibrating = true;
    static Vector3f accMean = Vector3f::Zero();
    static Vector3f magMean = Vector3f::Zero();

    auto data = bmx160->getLastSample();
    Vector3f acceleration(data.accelerationX, data.accelerationY,
                          data.accelerationZ);
    Vector3f angularSpeed(data.angularSpeedX, data.angularSpeedY,
                          data.angularSpeedZ);
    Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                           data.magneticFieldZ);

    // Apply correction
    Vector3f acc_b(0.3763 + 0.094, -0.1445 - 0.0229, -0.1010 + 0.0150);
    acceleration -= acc_b;
    Vector3f gyro_b{-1.63512255486542, 3.46523431469979, -3.08516033954451};
    angularSpeed = angularSpeed - gyro_b;
    angularSpeed = angularSpeed / 180 * Constants::PI / 10;
    Vector3f mag_b{21.0730033648425, -24.3997259703105, -2.32621524742862};
    Matrix3f A{{0.659926504672263, 0, 0},
               {0, 0.662442130094073, 0},
               {0, 0, 2.28747567094359}};
    magneticField = (magneticField - mag_b).transpose() * A;

    if (calibrating)
    {
        if (TimestampTimer::getTimestamp() < CALIBRATION_TIMEOUT)
        {
            accMean = (accMean * meanCount + acceleration) / (meanCount + 1);
            magMean = (magMean * meanCount + acceleration) / (meanCount + 1);
            meanCount++;
        }
        else
        {
            // Now the calibration has ended, compute and log the nas state
            calibrating = false;

            // Compute the initial nas state
            stateInitializer->triad(accMean, magMean, nedMag);
            nas->setX(stateInitializer->getInitX());

            // Save the state and the IMU data
            // Logger::getInstance().log(nas->getState());
            data.accelerationX  = accMean[0];
            data.accelerationY  = accMean[1];
            data.accelerationZ  = accMean[2];
            data.magneticFieldX = magMean[0];
            data.magneticFieldY = magMean[1];
            data.magneticFieldZ = magMean[2];
            // Logger::getInstance().log(data);

            // Restart the logger to change log filename
            // Logger::getInstance().stop();
            // Logger::getInstance().start();
        }
    }
    else
    {
        // Predict step
        {
            // nas->predictAcc(acceleration);
            nas->predictGyro(angularSpeed);

            data.angularSpeedX = angularSpeed[0];
            data.angularSpeedY = angularSpeed[1];
            data.angularSpeedZ = angularSpeed[2];
        }

        // Correct step
        {
            magneticField.normalize();
            nas->correctMag(magneticField);

            data.magneticFieldX = magneticField[0];
            data.magneticFieldY = magneticField[1];
            data.magneticFieldZ = magneticField[2];
        }

        auto nasState = nas->getState();

        nasState.timestamp          = TimestampTimer::getTimestamp();
        data.accelerationTimestamp  = nasState.timestamp;
        data.magneticFieldTimestamp = nasState.timestamp;
        data.angularSpeedTimestamp  = nasState.timestamp;

        // Logger::getInstance().log(nasState);
        // Logger::getInstance().log(data);
        printf("w%fwa%fab%fbc%fc\n", nasState.qw, nasState.qx, nasState.qy,
               nasState.qz);
    }
}
