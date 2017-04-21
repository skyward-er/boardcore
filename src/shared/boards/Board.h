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

enum AnakinSensor
{
    ACCEL_MPU_9250   = 0,
    GYRO_MPU_9250    = 1,
    MAGNETO_MPU_9250 = 2,
    TEMP_MPU_9250    = 3,
    DEBUG_MPU_9250   = 4,
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

typedef ProtocolSPI<busSPI1, CS_MPU9250> spiMPU9250;
typedef ProtocolSPI<busSPI1, CS_INEMO_A> spiINEMOA;
typedef ProtocolSPI<busSPI1, CS_INEMO_G> spiINEMOG;

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

        mpu_t* mpu9250 = new mpu_t(mpu_t::ACC_FS_16G, mpu_t::GYRO_FS_250);

        mpu9250->init();

        AddSensor(ACCEL_MPU_9250, DATA_VEC3, mpu9250->accelDataPtr());
        AddSensor(GYRO_MPU_9250, DATA_VEC3, mpu9250->gyroDataPtr());
        AddSensor(MAGNETO_MPU_9250, DATA_VEC3, mpu9250->compassDataPtr());
        AddSensor(TEMP_MPU_9250, DATA_FLOAT, mpu9250->tempDataPtr());
        AddSensor(DEBUG_MPU_9250, DATA_INT, mpu9250->debugIntPtr());

        printf("Adding sensors to 100Hz DMA sampler\n");
        m100HzDMA.AddSensor(mpu9250);
        printf("Adding sensors to 10Hz Simple sampler\n");
        m10HzSimple.AddSensor(mpu9250);

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

#define sBoard AnakinBoard::getInstance()

#endif /* BOARD_H */
