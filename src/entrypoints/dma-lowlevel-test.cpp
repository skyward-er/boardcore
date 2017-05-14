/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
 * Authors: Federico Terraneo
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include <sensors/iNemo.h>
#include <sensors/FXAS21002.h>
#include <sensors/MPU9250.h>
#include <sensors/LPS331AP.h>
#include <sensors/MAX21105.h>
#include <sensors/MS580301BA07.h>

#include <drivers/spi/SensorSpi.h>
#include <sensors/SensorSampling.h>

using namespace miosix;

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
    spiMPU9250::init();
    spiINEMOA::init();
    spiINEMOG::init();
    auto& spi=SPIDriver::instance();
    
    inemo_t inemo (inemo_t::ACC_FS_16G,
                   inemo_t::GYRO_FS_245,
                   inemo_t::COMPASS_FS_2);

    if(inemo.init()==false) 
    {
        puts("init failed");
    }

    auto req=inemo.buildDMARequest();
    auto sample=[&]()
    {
        if(spi.transaction(req)==false) puts("DMA error");
        for(auto& r : req)
        {
            inemo.onDMAUpdate(r);
            printf("ID: %d  --> ", r.id());
            auto& resp=r.readResponseFromPeripheral();
            memDump(resp.data(), resp.size());
        }
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
