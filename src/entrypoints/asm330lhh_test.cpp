#include "drivers/spi/SPIDriver.h"
#include "sensors/ASM330LHH/ASM330LHH.h"
#include "sensors/LIS3DSH/LIS3DSH.h"
#include <Common.h>


using namespace miosix;

SPIBus bus(SPI1); 

SPIBusConfig cfg_mysensor; // Bus configuration for the sensor

int main(){

    GpioPin cs(GPIOA_BASE, 1);          // Chip select pin of the sensor (GPIO A1) 
    GpioPin spi_ck(GPIOA_BASE, 5);      // SPI clock pin PA5
    GpioPin spi_out(GPIOA_BASE, 6);     // SPI output pin PA6
    GpioPin spi_in(GPIOA_BASE, 7);      // SPI input pin PA7

    {
        FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // SPI1 enable

        cs.mode(Mode::OUTPUT);
        spi_ck.mode(Mode::ALTERNATE);
        spi_ck.alternateFunction(5);
        spi_out.mode(Mode::ALTERNATE);
        spi_out.alternateFunction(5);
        spi_in.mode(Mode::ALTERNATE);
        spi_in.alternateFunction(5);
    }

    cs.high();

    asm330lhh_params params;

    params.accel_odr = ASM330LHH::ODR::_26HZ;
    params.gyro_odr = ASM330LHH::ODR::_26HZ;
    params.bdu = ASM330LHH::BDU::UPDATE_AFTER_READ;
    params.accel_fs = ASM330LHH::ACCEL_FS::_8G;
    params.gyro_fs = ASM330LHH::GYRO_FS::_250DPS;

    ASM330LHH sensor(bus, cs, params);
    bool success = sensor.init();

    if(success){
        TRACE("Init done\n");
    } else {
        TRACE("Init failed\n");
    }
}
