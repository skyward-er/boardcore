
#include <Common.h>
#include <BusTemplate.h>
#include <sensors/Si7021.h>
#include <Leds.h>
#include <util/software_i2c.h>

using namespace miosix;

typedef Gpio<GPIOB_BASE,7> sda;
typedef Gpio<GPIOB_BASE,8> scl;

typedef SoftwareI2C<sda,scl> i2c;

int main()
{
    Si7021<ProtocolI2C> hygro;
    i2c::init();
    printf("%d\n", hygro.selfTest() ? 1 : 0);
    
    uint16_t tmp;
    float temp;
    uint8_t buf[2];
    
    while(1)
    {
        hygro.updateParams();
        printf("temp %f, hum %f\n",hygro.getTemperature(),hygro.getHumidity());

        Thread::sleep(1000);
    }
    
    return 0;
}