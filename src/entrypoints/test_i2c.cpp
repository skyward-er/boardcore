
#include <Common.h>
#include <BusTemplate.h>
#include <sensors/MPL3115.h>
#include <Leds.h>
#include <drivers/stm32f2_f4_i2c.h>
// #include <util/software_i2c.h>

using namespace miosix;

int main()
{
    sensors::sda::mode(Mode::ALTERNATE_OD);
    sensors::sda::alternateFunction(4);
    sensors::scl::mode(Mode::ALTERNATE_OD);
    sensors::scl::alternateFunction(4);
    
    MPL3115<ProtocolI2C<I2C1Driver>> baro;
    
    printf("%d\n", baro.selfTest() ? 1 : 0);
    baro.setMode(baro.MODE_ALTIMETER);
    baro.setOversampleRatio(128);
    
    while(1)
    {
        baro.updateParams();
        printf("press %f, temp %f, alt %f\n",baro.getPressure(),baro.getTemperature(), baro.getAltitude());

        Thread::sleep(1000);
    }
    
    return 0;
}