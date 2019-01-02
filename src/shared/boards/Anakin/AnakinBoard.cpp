/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Federico Terraneo
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

#include "AnakinBoard.h"

#define INIT_AND_CHECK(x)                                \
    do                                                   \
    {                                                    \
        if (!x->init())                                  \
        {                                                \
            sLog->logString("=== ERR: CANNOT INIT " #x); \
        }                                                \
    } while (0)

AnakinBoard::AnakinBoard()
{
    mInited     = false;
    mS_MPU9250  = nullptr;
    mS_INEMO    = nullptr;
    mS_FXAS     = nullptr;
    mS_LPS331AP = nullptr;
    mS_MAX21105 = nullptr;
}

bool AnakinBoard::init()
{
    sLog->start();

    if (mInited)
        return false;

    spiMPU9250::init();
    spiINEMOA::init();
    spiINEMOG::init();
    spiFXAS21002::init();
    spiLPS331AP::init();
    spiMAX21105::init();
    spiMS580301BA07::init();

    // Raw sensors
    mInited = true;
    mS_MAX21105 =
        new max21105_t(max21105_t::ACC_FS_16G, max21105_t::GYRO_FS_250);
    mS_MPU9250 = new mpu_t(mpu_t::ACC_FS_16G, mpu_t::GYRO_FS_250);
    mS_INEMO   = new inemo_t(inemo_t::ACC_FS_16G, inemo_t::GYRO_FS_245,
                           inemo_t::COMPASS_FS_2);
    mS_FXAS     = new fxas_t(fxas_t::DPS500);
    mS_LPS331AP = new lps331ap_t(lps331ap_t::SS_25HZ);
    mS_MS580    = new ms580_t();

    // The MAX21105 has to be initialized first as it is prone to listening
    // to the initialization of the other sensors and switch by mistake to
    // I2C mode
    INIT_AND_CHECK(mS_MAX21105);
    INIT_AND_CHECK(mS_MPU9250);
    INIT_AND_CHECK(mS_INEMO);
    INIT_AND_CHECK(mS_FXAS);
    INIT_AND_CHECK(mS_LPS331AP);
    INIT_AND_CHECK(mS_MS580);

    // clang-format off
    AddSensor(ACCEL_MPU9250,    DATA_VEC3,     mS_MPU9250->accelDataPtr());
    AddSensor(GYRO_MPU9250,     DATA_VEC3,     mS_MPU9250->gyroDataPtr());
    AddSensor(COMPASS_MPU9250,  DATA_VEC3,     mS_MPU9250->compassDataPtr());
    AddSensor(TEMP_MPU9250,     DATA_FLOAT,    mS_MPU9250->tempDataPtr());

    AddSensor(ACCEL_INEMO,      DATA_VEC3,     mS_INEMO->accelDataPtr());
    AddSensor(GYRO_INEMO,       DATA_VEC3,     mS_INEMO->gyroDataPtr());
    AddSensor(COMPASS_INEMO,    DATA_VEC3,     mS_INEMO->compassDataPtr());
    AddSensor(TEMP_INEMO,       DATA_FLOAT,    mS_INEMO->tempDataPtr());

    AddSensor(GYRO_FXAS21002,   DATA_VEC3,     mS_FXAS->gyroDataPtr());

    AddSensor(PRESS_LPS331AP,   DATA_FLOAT,    mS_LPS331AP->pressureDataPtr());
    AddSensor(TEMP_LPS331AP,    DATA_FLOAT,    mS_LPS331AP->tempDataPtr());

    AddSensor(ACCEL_MAX21105,   DATA_VEC3,     mS_MAX21105->accelDataPtr());
    AddSensor(GYRO_MAX21105,    DATA_VEC3,     mS_MAX21105->gyroDataPtr());
    AddSensor(TEMP_MAX21105,    DATA_FLOAT,    mS_MAX21105->tempDataPtr());

    AddSensor(PRESS_MS580,      DATA_FLOAT,    mS_MS580->pressureDataPtr());
    AddSensor(TEMP_MS580,       DATA_FLOAT,    mS_MS580->tempDataPtr());
    // clang-format on

    sLog->logString("Adding sensors to 100Hz DMA sampler\n");
    m100HzDMA.AddSensor(mS_MPU9250);
    m100HzDMA.AddSensor(mS_INEMO);
    m100HzDMA.AddSensor(mS_FXAS);
    m100HzDMA.AddSensor(mS_MAX21105);

    sLog->logString("Adding sensors to 25Hz DMA sampler\n");
    m25HzDMA.AddSensor(mS_LPS331AP);

    sLog->logString("Adding sensors to 10Hz Simple sampler\n");
    m10HzSimple.AddSensor(mS_MPU9250);
    m10HzSimple.AddSensor(mS_MS580);

    sLog->logString("Adding samplers to scheduler\n");
#define ADD_SAMPLER(type, name, rate)                                         \
    sEventScheduler->add(std::bind(&type##SensorSampler::Update, name), rate, \
                         #name "-" #rate "ms", start)

    int64_t start = miosix::getTick();      // Synchronize activation
    ADD_SAMPLER(DMA, m100HzDMA, 10);        // 10ms
    ADD_SAMPLER(DMA, m25HzDMA, 40);         // 25ms
    ADD_SAMPLER(Simple, m10HzSimple, 100);  // 100ms

    return true;
}

#undef INIT_AND_CHECK
#undef ADD_SAMPLER
