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

enum AnakinSensor
{
    ACCEL_MPU_9250   = 0,
    GYRO_MPU_9250    = 1,
    MAGNETO_MPU_9250 = 2,
    TEMP_MPU_9250    = 3,
};

enum DataType
{
    DATA_VEC3  = 0,
    DATA_QUAT  = 1,
    DATA_FLOAT = 2,
};

struct SingleSensor
{
    AnakinSensor sensor;
    DataType data;
    void* value;

    SingleSensor() {}
    SingleSensor(AnakinSensor sensor, DataType data, void* value)
    {
        this->sensor = sensor;
        this->data = data;
        this->value = value;
    }
}

class Board 
{
public:
protected:
    std::vector<SingleSensor> mSensorData;
    std::vector<Sensor *> mRawSensors;

    void AddSensor(AnakinSensor sensor, DataType data, void* value)
    {
        mSensorData.push_back(SingleSensor(sensor,data,value)); 
    }
};

typedef Gpio<GPIOD_BASE, 13> CS_MPU9250;
typedef Gpio<GPIOG_BASE, 9> CS_INEMO_G;
typedef Gpio<GPIOG_BASE, 11> CS_INEMO_A;

typedef ProtocolSPI<busSPI1, CS_MPU9250> spiMPU9250;
typedef ProtocolSPI<busSPI1, CS_INEMO_A> spiINEMOA;
typedef ProtocolSPI<busSPI1, CS_INEMO_G> spiINEMOG;

class AnakinBoard : public Singleton<AnakinBoard>, public Board
{
    friend class Singleton<AnakinBoard>;
public:
private:
    AnakinBoard()
    {
        typedef MPU9250<spiMPU9250> mpu_t;
        typedef iNEMOLSM9DS0<spiINEMOG,spiINEMOA> inemo_t;

        mpu_t mpu9250(mpu_t::ACC_FS_16G, mpu_t::GYRO_FS_250);

        mpu9250.init();

        AddSensor(ACCEL_MPU_9250, DATA_VEC3, mpu9250.accelDataPtr());
        AddSensor(GYRO_MPU_9250, DATA_VEC3, mpu9250.gyroDataPtr());
        AddSensor(MAGNETO_MPU_9250, DATA_VEC3, mpu9250.magnetoDataPtr());
        AddSensor(TEMP_MPU_9250, DATA_FLOAT, mpu9250.tempDataPtr());
    }
}

#define sBoard AnakinBoard::getInstance()

#endif /* BOARD_H */
