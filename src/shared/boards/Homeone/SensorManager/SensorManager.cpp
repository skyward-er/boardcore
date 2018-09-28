/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SensorManager.h"

#include <libs/mavlink_skyward_lib/mavlink_lib/skyward/mavlink.h>

#include "TestSensor.h"
#include "boards/Homeone/Events.h"
#include "boards/Homeone/TMTCManager/TMTCManager.h"
#include "events/Scheduler.h"

#include "drivers/adc/AD7994.h"
#include "sensors/ADIS16405.h"
#include "sensors/MAX21105.h"
#include "sensors/MPU9250.h"

using miosix::Lock;
using miosix::FastMutex;

namespace HomeoneBoard
{
namespace Sensors
{

SensorManager::SensorManager() : EventHandler(), log(Logger::instance())
{
    sEventBroker->subscribe(this, TOPIC_CONFIGURATION);

    initSensors();
    initSamplers();
}

void SensorManager::initSensors()
{
    sensor_test = new TestSensor();  // TODO: Remove this

    adc_ad7994 = new AD7994Type(AD7994_I2C_ADDRESS);

    imu_max21105 =
        new MAX21105Type(0, 0);  // TODO: Update with correct parameters
    imu_max21105->init();

    imu_mpu9250 =
        new MPU9250Type(0, 0);  // TODO: Update with correct parameters
    imu_mpu9250->init();

    imu_adis16405 = new ADIS16405Type();
    imu_adis16405->init();
}

void SensorManager::initSamplers()
{
    sampler_20hz_simple.AddSensor(sensor_test);
    sampler_20hz_simple.AddSensor(adc_ad7994);

    sampler_500hz_dma.AddSensor(imu_max21105);
    sampler_500hz_dma.AddSensor(imu_mpu9250);
    sampler_500hz_dma.AddSensor(imu_adis16405);
}

void SensorManager::handleEvent(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_START_SAMPLING:
            startSampling();
            break;
        case EV_STOP_SAMPLING:
            // TODO: stop the samplers or just stop logging
            break;
        default:
            printf("Unrecognized event\n");
    }
}

SensorData SensorManager::getSensorData()
{
    SensorData data;
    data.testSensorData = *(sensor_test->testDataPtr());
    return data;
}

void SensorManager::startSampling()
{
    /*
     * std::bind syntax:
     * std::bind(&MyClass::someFunction, &myclass_instance, [someFunction args])
     */

    // Simple 20 Hz Sampler callback and scheduler function
    std::function<void()> simple_20hz_callback =
        std::bind(&SensorManager::onSimple20HZCallback, this);

    std::function<void()> simple_20hz_sampler =
        std::bind(&SimpleSensorSampler::UpdateAndCallback, &sampler_20hz_simple,
                  simple_20hz_callback);

    sEventScheduler->add(simple_20hz_sampler, 50, "simple_20hz");

    // DMA 500 Hz Sampler callback and scheduler function
    std::function<void()> dma_500hz_callback =
        std::bind(&SensorManager::onDMA500HZCallback, this);

    std::function<void()> dma_500Hz_sampler =
        std::bind(&DMASensorSampler::UpdateAndCallback, &sampler_500hz_dma,
                  dma_500hz_callback);

    sEventScheduler->add(dma_500Hz_sampler, 2, "dma_500hz");

    // Finally add TMTC send tasks
    std::function<void()> tmtc_low_rate =
        std::bind(&SensorManager::lowRateTMTC, this);
    sEventScheduler->add(tmtc_low_rate, 1000, "tmtc_lr");

    std::function<void()> tmtc_high_rate =
        std::bind(&SensorManager::lowRateTMTC, this);
    sEventScheduler->add(tmtc_high_rate, 100, "tmtc_hr");
}

void SensorManager::onSimple20HZCallback()
{
    // This is just a test
    printf("%f\n", *(sensor_test->testDataPtr()));

    // TODO: Send samples to logger
    // TODO: Send pressure samples to FMM
}

void SensorManager::onDMA500HZCallback()
{
    log.log(*(imu_max21105->gyroDataPtr()));
    log.log(*(imu_max21105->accelDataPtr()));
    log.log(*(imu_max21105->tempDataPtr()));

    log.log(*(imu_mpu9250->gyroDataPtr()));
    log.log(*(imu_mpu9250->accelDataPtr()));
    log.log(*(imu_mpu9250->tempDataPtr()));

    log.log(*(imu_adis16405->gyroDataPtr()));
    log.log(*(imu_adis16405->accelDataPtr()));
    log.log(*(imu_adis16405->tempDataPtr()));
}

void SensorManager::lowRateTMTC()
{
    mavlink_message_t low_rate_samples_msg;

    Vec3 gyro         = *(imu_adis16405->gyroDataPtr());
    Vec3 accel        = *(imu_adis16405->accelDataPtr());
    uint16_t pressure = 0;  // Use actual data

    mavlink_msg_sample_data_tm_pack(1, 1, &low_rate_samples_msg, pressure,
                                    accel.getX(), accel.getY(), accel.getZ(),
                                    gyro.getX(), gyro.getY(), gyro.getZ());

    int msg_len =
        mavlink_msg_to_send_buffer(tmtc_buffer, &low_rate_samples_msg);

    sTMTCManager->enqueueMsg(tmtc_buffer, msg_len);
}

void SensorManager::highRateTMTC()
{
    mavlink_message_t high_rate_samples_msg;

    uint16_t pressure = 0;  // Use actual data

    mavlink_msg_hr_sample_data_tm_pack(1, 1, &high_rate_samples_msg, pressure);
    int msg_len =
        mavlink_msg_to_send_buffer(tmtc_buffer, &high_rate_samples_msg);

    sTMTCManager->enqueueMsg(tmtc_buffer, msg_len);
}
}
}
