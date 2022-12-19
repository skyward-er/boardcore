#include <sensors/LSM6DSRX/LSM6DSRX.h>

#include <miosix.h>
#include <utils/Debug.h>
#include <drivers/spi/SPIDriver.h>


using namespace Boardcore;
using namespace miosix;

int main()
{
    SPIBus bus(SPI3);


    GpioPin csPin(GPIOE_BASE, 3);//PE3 CS
    csPin.mode(Mode::OUTPUT);

    GpioPin clockPin(GPIOB_BASE, 3);//PB3 CK (SCL)
    clockPin.mode(Mode::ALTERNATE);
    clockPin.alternateFunction(6);
    GpioPin misoPin(GPIOB_BASE, 4);//PB4 MISO (SDO)
    misoPin.alternateFunction(6);
    misoPin.mode(Mode::ALTERNATE);
    GpioPin mosiPin(GPIOB_BASE, 5);//PB5 MOSI (SDA)
    mosiPin.alternateFunction(6);
    mosiPin.mode(Mode::ALTERNATE);


    SPIBusConfig busConfiguration; // Bus configuration for the sensor
    busConfiguration.clockDivider = SPI::ClockDivider::DIV_256;
    busConfiguration.mode = SPI::Mode::MODE_0; // Set clock polarity to 0 and phase to 1


    LSM6DSRX sens(bus, csPin, busConfiguration);
    
    
    bool isInit = sens.init();
    while(true)
    {
        if(isInit == true)
        {
            TRACE("Correct WHO_AM_I\n\n");
        }
        else
        {
            TRACE("Error WHO_AM_I\n\n");
        }

        Thread::sleep(1000);
    }

    return 0;
}