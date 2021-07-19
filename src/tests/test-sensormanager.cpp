#include <assert.h>

#include <functional>
#include <iostream>
#include <type_traits>

#include "TimestampTimer.h"
#include "miosix.h"
#include "sensors/Sensor.h"
#include "sensors/SensorManager.h"

using namespace miosix;
using namespace TimestampTimer;

// Data produced by the MySensor sensor.
struct MySensorData : public PressureData, public TemperatureData
{

    MySensorData() : PressureData{0, 0.0}, TemperatureData{0, 0.0} {}

    MySensorData(float p, float t)
        : PressureData{getTimestamp(), p}, TemperatureData{getTimestamp(), t}
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
    MyPressureFilter(Sensor<SensorData>* original_sensor, float offset)
        : original_sensor(original_sensor), offset(offset)
    {
    }

    bool init() override { return true; };
    bool selfTest() override { return true; };

    SensorData sampleImpl() { return SensorData(); }

    const SensorData& getLastSample() override
    {
        this->last_sample = original_sensor->getLastSample();
        this->last_sample.press += offset;
        return this->last_sample;
    }

private:
    Sensor<SensorData>* original_sensor;
    float offset;
};

struct MySensorDataFIFO : public AccelerometerData, public GyroscopeData
{

    MySensorDataFIFO()
        : AccelerometerData{getTimestamp(), 0.0, 0.0, 0.0}, GyroscopeData{
                                                                getTimestamp(),
                                                                0.0, 0.0, 0.0}
    {
    }

    MySensorDataFIFO(AccelerometerData acc, GyroscopeData gyro)
        : AccelerometerData{getTimestamp(), acc.accel_x, acc.accel_y,
                            acc.accel_z},
          GyroscopeData{getTimestamp(), gyro.gyro_x, gyro.gyro_y, gyro.gyro_z}
    {
    }
};

class MySensorFIFO : public SensorFIFO<MySensorDataFIFO, 20>
{
    uint32_t fifo_size = 20;

public:
    MySensorFIFO() {}

    // return last sample
    virtual MySensorDataFIFO sampleImpl() override
    {
        for (uint32_t i = 0; i < fifo_size; i++)
        {
            AccelerometerData acc{getTimestamp(), 0.5, 0.5, 0.5};
            GyroscopeData gyro{getTimestamp(), 0.5, 0.5, 0.5};

            last_fifo[i] = MySensorDataFIFO{acc, gyro};

            TRACE("Accel : %llu %f %f %f \n", acc.accel_timestamp, acc.accel_x,
                  acc.accel_y, acc.accel_z);
            TRACE("Gyro : %llu %f %f %f \n", gyro.gyro_timestamp, gyro.gyro_x,
                  gyro.gyro_y, gyro.gyro_z);

            Thread::sleep(5);
        }

        last_fifo_level = fifo_size;

        return last_fifo[last_fifo_level - 1];
    }

    bool init() override { return true; };
    bool selfTest() override { return true; };
};

template <typename FIFOData, uint32_t FifoSize>
class FIFOProxy : public SensorFIFO<FIFOData, FifoSize>
{
public:
    FIFOProxy(SensorFIFO<FIFOData, FifoSize>* sensor) : sensor(sensor) {}

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

int main()
{
    srand(time(NULL));

    enableTimestampTimer();

    MySensor s1;
    MySensor s2;

    MyPressureFilter<MySensorData> filter(&s1, 2.578f);

    SensorManager SM(
        {{/*Sensor=*/&s1,
          {/*ID=*/"s1",
            /*Freq=*/1000,
           /*Callback=*/[]() { std::cout << "Callback s1: Hi 1!" << endl; },
           /*DMA=*/false,
           /*Enabled=*/true}},
         {/*Sensor=*/&s2,
          {/*ID=*/"s2",
            /*Freq=*/1000,
           /*Callback=*/[]() { std::cout << "Callback s2: Hi 2!" << endl; },
           /*DMA=*/false,
           /*Enabled=*/true}},
         {/*Sensor=*/&filter,
          {/*ID=*/"filter",
            /*Freq=*/2000,
           /*Callback=*/
           []() { std::cout << "Callback filter: Hi filter!" << endl; },
           /*DMA=*/false,
           /*Enabled=*/true}}});

    SM.start();

    Thread::sleep(1000);

    // TEST NORMAL SENSORS

    SM.disableSensor(&s1);

    Thread::sleep(1000);

    SM.enableSensor(&s1);

    for (int i = 0; i < 5; i++)
    {
        TRACE("S1 : %llu %f %llu %f \n", s1.getLastSample().press_timestamp,
              s1.getLastSample().press, s1.getLastSample().temp_timestamp,
              s1.getLastSample().temp);
        TRACE("Filter : %llu %f \n", filter.getLastSample().press_timestamp,
              filter.getLastSample().press);

        Thread::sleep(1000);
    }

    // TEST SENSORS WITH FIFO

    const uint32_t fifo_size = 20;

    MySensorFIFO s;

    FIFOProxy<MySensorDataFIFO, fifo_size> fifo_proxy(&s);

    for (int i = 0; i < 15; i++)
    {
        s.sample();

        MySensorDataFIFO data = fifo_proxy.getLastSample();

        TRACE("AccelProxy : %llu %f %f %f \n", data.accel_timestamp,
              data.accel_x, data.accel_y, data.accel_z);
        TRACE("GyroProxy : %llu %f %f %f \n", data.gyro_timestamp, data.gyro_x,
              data.gyro_y, data.gyro_z);

        Thread::sleep(1000);
    }

    return 0;
}