#include <assert.h>

#include <functional>
#include <iostream>
#include <type_traits>

#include "miosix.h"
#include "sensors/Sensor.h"
#include "sensors/SensorManager.h"

using namespace miosix;

// Data produced by the MySensor sensor.
struct MySensorData : public TimestampData,
                      public PressureData,
                      public TemperatureData
{

    MySensorData()
        : TimestampData{getTick()}, PressureData{0}, TemperatureData{0}
    {
    }

    MySensorData(float press, float temp)
        : TimestampData{getTick()}, PressureData{press}, TemperatureData{temp}
    {
    }
};

// Specific Sensor implementation.
class MySensor : public Sensor<MySensorData>
{
public:
    virtual MySensorData sampleImpl() override
    {
        return MySensorData(rand() % 100, /*temp=*/rand() % 100);
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

    SensorData getData() override
    {
        this->data = original_sensor->getData();
        this->data.pressure += offset;
        return this->data;
    }

private:
    Sensor<SensorData>* original_sensor;
    float offset;
};

int main()
{
    srand(time(NULL));

    MySensor s1;
    MySensor s2;

    MyPressureFilter<MySensorData> filter(&s1, 2.578f);

    SensorManager SM(
        {{/*Sensor=*/&s1,
          {/*Freq=*/1,
           /*Callback=*/[]() { std::cout << "Callback s1: Hi 1!" << endl; },
           /*DMA=*/false,
           /*Enabled=*/true}},
         {/*Sensor=*/&s2,
          {/*Freq=*/1,
           /*Callback=*/[]() { std::cout << "Callback s2: Hi 2!" << endl; },
           /*DMA=*/false,
           /*Enabled=*/true}},
         {/*Sensor=*/&filter,
          {/*Freq=*/1,
           /*Callback=*/
           []() { std::cout << "Callback filter: Hi filter!" << endl; },
           /*DMA=*/false,
           /*Enabled=*/true}}});

    SM.start();

    Thread::sleep(1000);

    SM.disableSensor(&s1);

    Thread::sleep(1000);

    SM.enableSensor(&s1);

    while (1)
    {
        TRACE("S1 : %llu %f %f \n", s1.getData().timestamp,
              s1.getData().pressure, s1.getData().temperature);
        TRACE("Filter : %llu %f \n", filter.getData().timestamp,
              filter.getData().pressure);

        Thread::sleep(1000);
    }

    return 0;
}