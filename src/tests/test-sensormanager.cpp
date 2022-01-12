/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <assert.h>
#include <drivers/timer/TimestampTimer.h>
#include <sensors/Sensor.h>
#include <sensors/SensorManager.h>
#include <utils/Debug.h>
#include <utils/testutils/TestSensor.h>

#include <functional>
#include <iostream>
#include <type_traits>

#include "miosix.h"

using namespace Boardcore;
using namespace miosix;
using namespace std;

// Data produced by the MySensor sensor.
struct MySensorData : public PressureData, public TemperatureData
{

    MySensorData() : PressureData{0, 0.0}, TemperatureData{0, 0.0} {}

    MySensorData(float p, float t)
        : PressureData{TimestampTimer::getInstance().getTimestamp(), p},
          TemperatureData{TimestampTimer::getInstance().getTimestamp(), t}
    {
    }
};

// Specific Sensor implementation.
class MySensor : public Sensor<MySensorData>
{
public:
    virtual MySensorData sampleImpl() override
    {
        return MySensorData(rand() % 100, rand() % 100);
    }

    bool init() override { return true; };
    bool selfTest() override { return true; };
};

template <typename SensorData>
class MyPressureFilter : public Sensor<SensorData>
{
    static_assert(
        checkIfProduces<Sensor<SensorData>, PressureData>::value,
        "Template argument must be a sensor that produces pressure data.");

public:
    MyPressureFilter(Sensor<SensorData>* originalSensor, float offset)
        : originalSensor(originalSensor), offset(offset)
    {
    }

    bool init() override { return true; };
    bool selfTest() override { return true; };

    SensorData sampleImpl() { return SensorData(); }

    const SensorData& getLastSample() override
    {
        this->lastSample = originalSensor->getLastSample();
        this->lastSample.pressure += offset;
        return this->lastSample;
    }

private:
    Sensor<SensorData>* originalSensor;
    float offset;
};

struct MySensorDataFIFO : public AccelerometerData, public GyroscopeData
{

    MySensorDataFIFO()
        : AccelerometerData{TimestampTimer::getInstance().getTimestamp(), 0.0,
                            0.0, 0.0},
          GyroscopeData{TimestampTimer::getInstance().getTimestamp(), 0.0, 0.0,
                        0.0}
    {
    }

    MySensorDataFIFO(AccelerometerData acc, GyroscopeData gyro)
        : AccelerometerData{TimestampTimer::getInstance().getTimestamp(),
                            acc.accelerationX, acc.accelerationY,
                            acc.accelerationZ},
          GyroscopeData{TimestampTimer::getInstance().getTimestamp(),
                        gyro.angularVelocityX, gyro.angularVelocityY,
                        gyro.angularVelocityZ}
    {
    }
};

class MySensorFIFO : public SensorFIFO<MySensorDataFIFO, 20>
{
    uint32_t fifoSize = 20;

public:
    MySensorFIFO() {}

    // return last sample
    virtual MySensorDataFIFO sampleImpl() override
    {
        for (uint32_t i = 0; i < fifoSize; i++)
        {
            AccelerometerData acc{TimestampTimer::getInstance().getTimestamp(),
                                  0.5, 0.5, 0.5};
            GyroscopeData gyro{TimestampTimer::getInstance().getTimestamp(),
                               0.5, 0.5, 0.5};

            lastFifo[i] = MySensorDataFIFO{acc, gyro};

            TRACE("Accel : %llu %f %f %f \n", acc.accelerationTimestamp,
                  acc.accelerationX, acc.accelerationY, acc.accelerationZ);
            TRACE("Gyro : %llu %f %f %f \n", gyro.angularVelocityTimestamp,
                  gyro.angularVelocityX, gyro.angularVelocityY,
                  gyro.angularVelocityZ);

            Thread::sleep(5);
        }

        lastFifoLevel = fifoSize;

        return lastFifo[lastFifoLevel - 1];
    }

    bool init() override { return true; };
    bool selfTest() override { return true; };
};

template <typename FIFOData, uint32_t FifoSize>
class FIFOProxy : public SensorFIFO<FIFOData, FifoSize>
{
public:
    explicit FIFOProxy(SensorFIFO<FIFOData, FifoSize>* sensor) : sensor(sensor)
    {
    }

    bool init() override { return true; };
    bool selfTest() override { return true; };

    FIFOData sampleImpl()
    {
        index = 0;
        return sensor->getFifoElement(index);
    }

    const FIFOData& getLastSample() override
    {
        if (index < FifoSize - 1)
        {
            index++;
        }

        TRACE("Index : %d \n", index);

        return sensor->getFifoElement(index);
    }

private:
    SensorFIFO<FIFOData, FifoSize>* sensor;
    uint32_t index = 0;
};

class FailingSensor : public Sensor<TestData>
{
    bool init() { return true; }

    bool selfTest()  // always fail self-test
    {
        TRACE("Failed to init sensor FailingSensor!\n");
        return false;
    }

    TestData sampleImpl() { return TestData{}; }
};

int main()
{
    srand(time(NULL));

    MySensor s1;
    MySensor s2;

    MyPressureFilter<MySensorData> filter(&s1, 2.578f);

    FailingSensor failigS;  // must not be initialized and not sampled

    SensorManager sm({{/*Sensor=*/&s1,
                       {/*ID=*/"s1",
                        /*Freq=*/1000,
                        /*Callback=*/[]() { cout << "Callback s1!" << endl; },
                        /*DMA=*/false,
                        /*Enabled=*/true}},
                      {/*Sensor=*/&s2,
                       {/*ID=*/"s2",
                        /*Freq=*/1000,
                        /*Callback=*/[]() { cout << "Callback s2!" << endl; },
                        /*DMA=*/false,
                        /*Enabled=*/true}},
                      {/*Sensor=*/&filter,
                       {/*ID=*/"filter",
                        /*Freq=*/2000,
                        /*Callback=*/
                        []() { cout << "Callback filter!" << endl; },
                        /*DMA=*/false,
                        /*Enabled=*/true}},
                      {/*Sensor=*/&failigS,
                       {/*ID=*/"failing",
                        /*Freq=*/3000,
                        /*Callback=*/
                        []() { cout << "Callback failing sensor!" << endl; },
                        /*DMA=*/false,
                        /*Enabled=*/true}}});

    sm.start();

    Thread::sleep(1000);

    // TEST NORMAL SENSORS

    sm.disableSensor(&s1);

    Thread::sleep(1000);

    sm.enableSensor(&s1);

    for (int i = 0; i < 3; i++)
    {
        TRACE("S1 : %llu %f %llu %f \n", s1.getLastSample().pressureTimestamp,
              s1.getLastSample().pressure,
              s1.getLastSample().temperatureTimestamp,
              s1.getLastSample().temperature);
        TRACE("Filter : %llu %f \n", filter.getLastSample().pressureTimestamp,
              filter.getLastSample().pressure);

        Thread::sleep(1000);
    }

    // TEST SENSORS WITH FIFO

    const uint32_t fifoSize = 20;  // cppcheck-suppress unreadVariable

    MySensorFIFO s;

    FIFOProxy<MySensorDataFIFO, fifoSize> fifoProxy(&s);

    for (int i = 0; i < 3; i++)
    {
        s.sample();

        MySensorDataFIFO data = fifoProxy.getLastSample();

        UNUSED(data);

        TRACE("AccelProxy : %llu %f %f %f \n", data.accelerationTimestamp,
              data.accelerationX, data.accelerationY, data.accelerationZ);
        TRACE("GyroProxy : %llu %f %f %f \n", data.angularVelocityTimestamp,
              data.angularVelocityX, data.angularVelocityY,
              data.angularVelocityZ);

        Thread::sleep(1000);
    }

    for (;;)
        ;

    return 0;
}
