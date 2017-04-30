
#include <sensors/iNemo.h>
#include <sensors/FXAS21002.h>
#include <sensors/MPU9250.h>
#include <sensors/LPS331AP.h>
#include <sensors/MAX21105.h>
#include <sensors/MS580301BA07.h>

#include <DMA/DMA.h>
#include <DMA/SensorSampling.h>

typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;
typedef BusSPI < 1, GpioMosi, GpioMiso, GpioSck> busSPI1;

typedef ProtocolSPI<busSPI1, Gpio<GPIOD_BASE, 13>> spiMPU9250;
typedef ProtocolSPI<busSPI1, Gpio<GPIOG_BASE, 11>> spiINEMOA;
typedef ProtocolSPI<busSPI1, Gpio<GPIOG_BASE,  9>> spiINEMOG;

typedef MPU9250<spiMPU9250> mpu_t;
typedef iNEMOLSM9DS0<spiINEMOG,spiINEMOA> inemo_t;

int main()
{
    puts("\n\n---");
    auto& spi=SPIDriver::instance();
    
    inemo_t inemo(inemo_t::ACC_FS_16G,inemo_t::GYRO_FS_245,inemo_t::COMPASS_FS_2);
    if(inemo.init()==false) puts("init failed");
    auto req=inemo.buildDMARequest();
    
    auto sample=[&]()
    {
        if(spi.transaction(req)==false) puts("DMA error");
        for(auto& r : req) inemo.onDMAUpdate(r);
    };
    
    for(int i=0;i<2;i++)
    {
        Thread::sleep(20);
        sample();
    }
    
    for(;;)
    {
        getchar();
        sample();
    }
}
