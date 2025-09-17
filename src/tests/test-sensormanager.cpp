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
#include <drivers/dma/DMA.h>
#include <drivers/timer/TimestampTimer.h>
#include <sensors/Sensor.h>
#include <sensors/SensorManager.h>
#include <utils/Debug.h>
#include <utils/TestUtils/TestSensor.h>

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
        : PressureData{TimestampTimer::getTimestamp(), p},
          TemperatureData{TimestampTimer::getTimestamp(), t}
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

class MySensorDMA : public Sensor<TestData>
{
public:
    MySensorDMA() : dmaStream(nullptr) {}

    // cppcheck-suppress  noExplicitConstructor
    MySensorDMA(DMAStreamGuard* s) : dmaStream(s)
    {
        source.reserve(size);
        dest.resize(size);

        for (int i = 0; i < size; ++i)
            source.push_back(i);
    }

    virtual TestData sampleImpl() override
    {
        if (dmaStream == nullptr)
        {
            lastError = SensorErrors::NOT_INIT;
            return {};
        }

        lastDmaError = DMAErrors::NO_ERRORS;

        DMATransaction trn = {
            .srcSize                         = DMATransaction::DataSize::BITS_8,
            .dstSize                         = DMATransaction::DataSize::BITS_8,
            .srcAddress                      = source.data(),
            .dstAddress                      = dest.data(),
            .numberOfDataItems               = size,
            .srcIncrement                    = true,
            .dstIncrement                    = true,
            .enableTransferCompleteInterrupt = true,
            .enableTransferErrorInterrupt    = true,
        };

        (*dmaStream)->setup(trn);
        (*dmaStream)->enable();
        bool outcome = (*dmaStream)->timedWaitForTransferComplete(timeout);

        if (!outcome)
            lastDmaError = DMAErrors::TIMEOUT;

        for (int i = 0; i < size; ++i)
        {
            if (dest[i] != source[i] && lastDmaError == DMAErrors::NO_ERRORS)
            {
                outcome      = false;
                lastDmaError = DMAErrors::TRANSFER_ERROR;
            }

            // Reset destination
            dest[i] = 0;
        }

        if (outcome)
            lastError = NO_ERRORS;
        else
            lastError = BUS_FAULT;  // Error

        return {};
    }

    bool init() override { return dmaStream != nullptr; };
    bool selfTest() override { return dmaStream != nullptr; };
    DMAErrors getDmaError() { return lastDmaError; };

private:
    static constexpr int size               = 10;
    const std::chrono::milliseconds timeout = std::chrono::milliseconds(200);

    DMAStreamGuard* dmaStream;
    std::vector<uint8_t> source;
    std::vector<uint8_t> dest;
    DMAErrors lastDmaError = DMAErrors::NO_ERRORS;
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

    SensorData getLastSample() override
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
        : AccelerometerData{TimestampTimer::getTimestamp(), 0.0, 0.0, 0.0},
          GyroscopeData{TimestampTimer::getTimestamp(), 0.0, 0.0, 0.0}
    {
    }

    MySensorDataFIFO(AccelerometerData acc, GyroscopeData gyro)
        : AccelerometerData{TimestampTimer::getTimestamp(), acc.accelerationX,
                            acc.accelerationY, acc.accelerationZ},
          GyroscopeData{TimestampTimer::getTimestamp(), gyro.angularSpeedX,
                        gyro.angularSpeedY, gyro.angularSpeedZ}
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
            AccelerometerData acc{TimestampTimer::getTimestamp(), 0.5, 0.5,
                                  0.5};
            GyroscopeData gyro{TimestampTimer::getTimestamp(), 0.5, 0.5, 0.5};

            lastFifo[i] = MySensorDataFIFO{acc, gyro};

            TRACE("Accel : %llu %f %f %f \n", acc.accelerationTimestamp,
                  acc.accelerationX, acc.accelerationY, acc.accelerationZ);
            TRACE("Gyro : %llu %f %f %f \n", gyro.angularSpeedTimestamp,
                  gyro.angularSpeedX, gyro.angularSpeedY, gyro.angularSpeedZ);

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
        uint16_t actualFifoSize;
        return sensor->getLastFifo(actualFifoSize)[index];
    }

    FIFOData getLastSample() override
    {
        if (index < FifoSize - 1)
            index++;

        TRACE("Index : %d \n", index);
        uint16_t actualFifoSize;
        return sensor->getLastFifo(actualFifoSize)[index];
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

    auto dmaStream = DMADriver::instance().acquireStreamForPeripheral(
        DMADefs::Peripherals::PE_MEM_ONLY);
    if (!dmaStream.isValid())
    {
        TRACE("Error, cannot acquire dma stream");
        return 1;
    }

    MySensorDMA sDma1(&dmaStream);
    MySensorDMA sDma2(&dmaStream);
    MySensorDMA sDmaFailing;

    MyPressureFilter<MySensorData> filter(&s1, 2.578f);

    FailingSensor failigS;  // must not be initialized and not sampled

    SensorManager sm({
        {/*Sensor=*/&s1,
         {/*ID=*/"s1",
          /*Freq=*/1000,
          /*Callback=*/[]() { cout << "Callback s1!" << endl; },
          /*Enabled=*/true}},
        {/*Sensor=*/&s2,
         {/*ID=*/"s2",
          /*Freq=*/1000,
          /*Callback=*/[]() { cout << "Callback s2!" << endl; },
          /*Enabled=*/true}},
        {/*Sensor=*/&filter,
         {/*ID=*/"filter",
          /*Freq=*/2000,
          /*Callback=*/
          []() { cout << "Callback filter!" << endl; },
          /*Enabled=*/true}},
        {/*Sensor=*/&failigS,
         {/*ID=*/"failing",
          /*Freq=*/3000,
          /*Callback=*/
          []() { cout << "Callback failing sensor!" << endl; },
          /*Enabled=*/true}},
        {/*Sensor=*/&sDma1,
         {/*ID=*/"sDma1",
          /*Freq=*/1000,
          /*Callback=*/
          [&sDma1]()
          {
              if (sDma1.getLastError() != NO_ERRORS)
                  cout << "Callback sDma1, lastDmaError: "
                       << (unsigned int)sDma1.getDmaError() << endl;
              else
                  cout << "Callback sDma1!" << endl;
          },
          /*Enabled=*/true,
          /*GroupId=*/1}},
        {/*Sensor=*/&sDma2,
         {/*ID=*/"sDma2",
          /*Freq=*/1000,
          /*Callback=*/
          [&sDma2]()
          {
              if (sDma2.getLastError() != NO_ERRORS)
                  cout << "Callback sDma2, lastDmaError: "
                       << (unsigned int)sDma2.getDmaError() << endl;
              else
                  cout << "Callback sDma2!" << endl;
          },
          /*Enabled=*/true,
          /*GroupId=*/1}},
        {/*Sensor=*/&sDmaFailing,
         {/*ID=*/"sDmaFailing",
          /*Freq=*/1000,
          /*Callback=*/
          []() { cout << "Callback sDmaFailing!" << endl; },
          /*Enabled=*/true,
          /*GroupId=*/1}},
    });

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

    const uint32_t fifoSize = 20;

    MySensorFIFO s;

    FIFOProxy<MySensorDataFIFO, fifoSize> fifoProxy(&s);

    for (int i = 0; i < 3; i++)
    {
        s.sample();

        MySensorDataFIFO data __attribute__((unused)) =
            fifoProxy.getLastSample();

        TRACE("AccelProxy : %llu %f %f %f \n", data.accelerationTimestamp,
              data.accelerationX, data.accelerationY, data.accelerationZ);
        TRACE("GyroProxy : %llu %f %f %f \n", data.angularSpeedTimestamp,
              data.angularSpeedX, data.angularSpeedY, data.angularSpeedZ);

        Thread::sleep(1000);
    }

    for (;;)
        ;

    return 0;
}
