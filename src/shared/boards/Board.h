/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci
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

#ifndef BOARD_H
#define BOARD_H

#include <Common.h>
#include <BusTemplate.h>
#include <Singleton.h>

#include <sensors/iNemo.h>
#include <sensors/FXAS21002.h>
#include <sensors/MPU9250.h>

enum AnakinSensor
{
    MPU_9250_ACCEL   = 0,
    MPU_9250_GYRO    = 1,
    MPU_9250_COMPASS = 2,
    MPU_9250_TEMP    = 3,

    INEMO_ACCEL      = 4,
    INEMO_GYRO       = 5,
    INEMO_COMPASS    = 6,
    INEMO_TEMP       = 7,
    FXAS_GYRO        = 8,
};

enum DataType
{
    DATA_VEC3  = 0,
    DATA_QUAT  = 1,
    DATA_FLOAT = 2,
    DATA_INT   = 3,
};

struct SingleSensor
{
    AnakinSensor sensor;
    DataType data;
    const void* value;

    SingleSensor() {}
    SingleSensor(AnakinSensor sensor, DataType data, const void* value) :
        sensor(sensor), data(data), value(value)
    { }
};

class Board 
{
public:
protected:
    std::vector<SingleSensor> mSensorData;
    std::vector<Sensor *> mRawSensors;

    void AddSensor(AnakinSensor sensor, DataType data, const void* value)
    {
        mSensorData.push_back(SingleSensor(sensor,data,value)); 
    }
};

typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;
typedef BusSPI < 1, GpioMosi, GpioMiso, GpioSck> busSPI1;

typedef Gpio<GPIOD_BASE, 13> CS_MPU9250;
typedef Gpio<GPIOG_BASE, 9> CS_INEMO_G;
typedef Gpio<GPIOG_BASE, 11> CS_INEMO_A;
typedef Gpio<GPIOG_BASE, 10> CS_FXAS21002;

typedef ProtocolSPI<busSPI1, CS_MPU9250> spiMPU9250;
typedef ProtocolSPI<busSPI1, CS_INEMO_A> spiINEMOA;
typedef ProtocolSPI<busSPI1, CS_INEMO_G> spiINEMOG;
typedef ProtocolSPI<busSPI1, CS_FXAS21002> spiFXAS21002;

#define INIT_AND_CHECK(x) do {                                      \
    if (!x->init()) { printf("=== ERR: CANNOT INIT " #x "\n"); }    \
} while(0)

#define ADD_SAMPLER(type, name, rate) \
    sEventScheduler->add(std::bind(& type ## SensorSampler::Update,name), rate, #rate "ms")
class AnakinBoard : public Singleton<AnakinBoard>, public Board
{
    friend class Singleton<AnakinBoard>;
public:
    void init()
    {
        if(mInited)
            return;
        mInited = true;

        typedef MPU9250<spiMPU9250> mpu_t;
        typedef iNEMOLSM9DS0<spiINEMOG,spiINEMOA> inemo_t;
        typedef FXAS21002<spiFXAS21002> fxas_t;

        mpu_t* mpu9250 = new mpu_t(mpu_t::ACC_FS_2G, mpu_t::GYRO_FS_250);
        inemo_t* iNemo = new inemo_t(inemo_t::ACC_FS_16G, inemo_t::GYRO_FS_245,
                                     inemo_t::COMPASS_FS_2);
        fxas_t* fxas   = new fxas_t(fxas_t::DPS500);

        INIT_AND_CHECK(mpu9250);
        INIT_AND_CHECK(iNemo);
        INIT_AND_CHECK(fxas);

        AddSensor(MPU_9250_ACCEL,   DATA_VEC3, mpu9250->accelDataPtr());
        AddSensor(MPU_9250_GYRO,    DATA_VEC3, mpu9250->gyroDataPtr());
        AddSensor(MPU_9250_COMPASS, DATA_VEC3, mpu9250->compassDataPtr());
        AddSensor(MPU_9250_TEMP,    DATA_FLOAT, mpu9250->tempDataPtr());

        AddSensor(INEMO_ACCEL,      DATA_VEC3, iNemo->accelDataPtr());
        AddSensor(INEMO_GYRO,       DATA_VEC3, iNemo->gyroDataPtr());
        AddSensor(INEMO_COMPASS,    DATA_VEC3, iNemo->compassDataPtr());
        AddSensor(INEMO_TEMP,       DATA_FLOAT, iNemo->tempDataPtr());
        AddSensor(FXAS_GYRO,        DATA_VEC3, fxas->gyroDataPtr());

        printf("Adding sensors to 100Hz DMA sampler\n");
        m100HzDMA.AddSensor(mpu9250);
        m100HzDMA.AddSensor(iNemo);
        m100HzDMA.AddSensor(fxas);

        printf("Adding sensors to 10Hz Simple sampler\n");
        m10HzSimple.AddSensor(mpu9250);
        m10HzSimple.AddSensor(fxas);

        printf("Adding samplers to scheduler\n");
        ADD_SAMPLER(DMA, m100HzDMA, 10); // 10ms
        ADD_SAMPLER(DMA, m10HzDMA, 100); // 100ms
        ADD_SAMPLER(Simple, m100HzSimple, 10); // 10ms
        ADD_SAMPLER(Simple, m10HzSimple, 100); // 100ms
    }

    const std::vector<SingleSensor>& debugGetSensors() const
    {
        return mSensorData;
    }

private:
    DMASensorSampler m100HzDMA, m10HzDMA;
    SimpleSensorSampler m100HzSimple, m10HzSimple;
    bool mInited;
    AnakinBoard()
    {
        mInited = false;
    }
};

#undef INIT_AND_CHECK
#undef ADD_SAMPLER

#define sBoard AnakinBoard::getInstance()

#endif /* BOARD_H */
