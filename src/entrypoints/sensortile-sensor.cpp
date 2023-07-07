#include <logger/Logger.h>
#include <miosix.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <utils/Debug.h>

using namespace miosix;
using namespace Boardcore;

Logger& logger = Logger::getInstance();

GpioPin clk(GPIOB_BASE, 3);
GpioPin miso(GPIOB_BASE, 4);
GpioPin mosi(GPIOB_BASE, 5);
GpioPin cs(GPIOA_BASE, 15);

int main()
{
    // start logger
    TRACE("Setting up Logger...\n");
    if (!logger.start())
    {
        TRACE("Logger failed to start, aborting.\n");
        return 1;
    }

    TRACE("Setting up Pins...\n");

    clk.mode(Mode::ALTERNATE);
    clk.alternateFunction(5);
    clk.speed(Speed::_100MHz);
    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);
    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);
    cs.mode(Mode::OUTPUT);
    cs.high();

    TRACE("Setting up SPI...\n");
    SPIBus bus(SPI3);

    SPIBusConfig busConfig = LIS2MDL::getDefaultSPIConfig();

    LIS2MDL::Config config;
    config.odr                = LIS2MDL::ODR_10_HZ;
    config.deviceMode         = LIS2MDL::MD_CONTINUOUS;
    config.temperatureDivider = 5;

    TRACE("Setting up Sensor...\n");
    LIS2MDL sensor(bus, cs, busConfig, config);

    if (!sensor.init())
    {
        TRACE("LIS2MDL: Init failed\n");
        return 1;
    }
    TRACE("LIS2MDL: Init done\n");

    TRACE("Doing self test!\n");
    if (!sensor.selfTest())
    {
        TRACE("Error: selfTest() returned false!\n");
    }
    TRACE("selfTest returned true\n");
    TRACE("Now printing some sensor data:\n");

    for (int i = 0; i < 10; i++)
    {
        sensor.sample();
        LIS2MDLData data = sensor.getLastSample();
        TRACE("%f C | x: %f | y: %f | z: %f\n", data.temperature,
               data.magneticFieldX, data.magneticFieldY, data.magneticFieldZ);
        miosix::Thread::sleep(10);
    }

    TRACE("Completed\n");
    logger.stop();
    return 0;
}
