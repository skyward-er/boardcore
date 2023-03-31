#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <utils/Debug.h>
#include "miosix.h"

using namespace Boardcore;
using namespace miosix;

SPIBus bus(SPI1);

GpioPin spiSck(GPIOA_BASE, 5);
GpioPin spiMiso(GPIOA_BASE, 6);
GpioPin spiMosi(GPIOA_BASE, 7);
GpioPin cs(GPIOE_BASE, 4);

int main()
{
    Thread::sleep(1000);

    spiSck.mode(miosix::Mode::ALTERNATE);
    spiSck.alternateFunction(5);
    spiMiso.mode(miosix::Mode::ALTERNATE);
    spiMiso.alternateFunction(5);
    spiMosi.mode(miosix::Mode::ALTERNATE);
    spiMosi.alternateFunction(5);

    cs.mode(miosix::Mode::OUTPUT);
    cs.high();

    H3LIS331DL sensor(bus, cs, sensor.ODR_50, sensor.BDU_CONTINUOS_UPDATE,
                      sensor.FS_400);

    H3LIS331DLData data;

    if (!sensor.init())
    {
        printf("Failed init!\n");
        if (sensor.getLastError() == SensorErrors::INVALID_WHOAMI)
        {
            printf("Invalid WHOAMI\n");
        }
        return -1;
    }

    // Print out the CSV header
    printf(H3LIS331DLData::header().c_str());
    // sample some data from the sensor
    for (int i = 0; i < 255; i++)
    {
        // sensor intitialized, should return error if no new data exist
        sensor.sample();

        // if (sensor.getLastError() == SensorErrors::NO_NEW_DATA)
        // {
        //     printf("\nWarning: no new data to be read \n");
        // }

        data = sensor.getLastSample();

        printf("%llu,%f,%f,%f\n", data.accelerationTimestamp,
               data.accelerationX, data.accelerationY, data.accelerationZ);

        Thread::sleep(100);
    }

    return 0;
}
