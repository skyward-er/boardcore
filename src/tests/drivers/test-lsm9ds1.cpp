/**
 * test LSM9DS1 axel + gyro
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Andrea Milluzzo
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


#include "drivers/spi/SPIDriver.h"
#include "sensors/LSM9DS1/LSM9DS1_AxelGyro.h"

using namespace miosix;

typedef Gpio<GPIOA_BASE, 5> GpioSck; //questi sono i pin SPI per f407_discovery
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;

static const bool FIFO_ENABLED = false;
Vec3 adata, gdata;
float tdata; 

//SPI
SPIBus bus(SPI1);
SPIBusConfig cfg;
GpioPin cs(GPIOE_BASE, 7);



int main(){

    cfg.clock_div=SPIClockDivider::DIV64;

    {
        FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //SPI1 ENABLE
        
        GpioSck::mode(Mode::ALTERNATE);
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMosi::mode(Mode::ALTERNATE);
        cs.mode(Mode::OUTPUT);

        GpioSck::alternateFunction(5);
        GpioMiso::alternateFunction(5);
        GpioMosi::alternateFunction(5);

        GpioSck::speed(Speed::_25MHz); 

    }

    cs.high();
    LSM9DS1_XLG lsm9ds1(
                    bus,
                    cs,
                    LSM9DS1_XLG::AxelFSR::FS_8, 
                    LSM9DS1_XLG::GyroFSR::FS_245,
                    LSM9DS1_XLG::ODR::ODR_952
                    );

    while(!lsm9ds1.init()){}
    Thread::sleep(500);
    printf("ax,ay,az,gx,gy,gz,t\n");
    long long first_tick = getTick();  
    for(;;)
    {   
        long long last_tick = getTick(); 
        lsm9ds1.onSimpleUpdate();
        adata = *(lsm9ds1.accelDataPtr());
        gdata = *(lsm9ds1.gyroDataPtr());
        tdata = *(lsm9ds1.tempDataPtr());
        printf("%d;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.1f\n",
                (last_tick - first_tick), 
                adata.getX(), adata.getY(), adata.getZ(),
                gdata.getX(), gdata.getY(), gdata.getZ(),
                tdata);
    }
    
    
    return 0;
}