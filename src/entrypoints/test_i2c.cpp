
#include <Common.h>
#include <BusTemplate.h>
#include <sensors/MPL3115.h>
#include <Leds.h>
// #include <util/software_i2c.h>

using namespace miosix;

typedef Gpio<GPIOB_BASE,7> sda;
typedef Gpio<GPIOB_BASE,8> scl;

// typedef SoftwareI2C<sda,scl> i2c;

int main()
{
    MPL3115<ProtocolI2C> baro;
    
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