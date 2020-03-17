/**
 * test LSM9DS1 magneto
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
#include "sensors/LSM9DS1/LSM9DSI_Magneto.h"

#define MAX_ADDR 0x33

using namespace miosix;

typedef Gpio<GPIOA_BASE, 5> GpioSck; //pin SPI1 per f407_discovery
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;


//GpioPin PushButton(GPIOA_BASE, 0);
GpioPin LED(GPIOD_BASE, 15);

Vec3 mdata;

//SPI
SPIBus bus(SPI1);
SPIBusConfig cfg;
GpioPin cs_M(GPIOE_BASE, 8);

int main()
{
    cfg.clock_div=SPIClockDivider::DIV64;
    {
        FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //SPI1 ENABLE

        GpioSck::mode(Mode::ALTERNATE);
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMosi::mode(Mode::ALTERNATE);

        cs_M.mode(Mode::OUTPUT);

        GpioSck::alternateFunction(5);
        GpioMiso::alternateFunction(5);
        GpioMosi::alternateFunction(5);

        GpioSck::speed(Speed::_25MHz);

        //PushButton.mode(Mode::INPUT);
        LED.mode(Mode::OUTPUT);
    }

    cs_M.high(); 

    Thread::sleep(200);
    
    //dump&test registers
    /*while(1)
    {
        if(PushButton.value())
        {
            LED.high();

            printf("\n\nREGISTER SET BEFORE INIT:\n\n");
            uint8_t regset1[0x34]; 
            for(int addr=0; addr<=MAX_ADDR; addr++)
            {
                SPISlave spislave(bus, cs_M, cfg);
                SPITransaction spi(spislave);
                regset1[addr] = spi.read(addr);
                printf("0x%02X-->0x%02X\n", addr,regset1[addr]);
            }

            LSM9DS1_M lsm9ds1(
                        bus,
                        cs_M,
                        LSM9DS1_M::MagFSR::FS_12,
                        LSM9DS1_M::ODR::ODR_40
                        );

            lsm9ds1.init(); 

            printf("\n\nREGISTER SET AFTER INIT:\n\n");
            uint8_t regset2[0x34]; 
            for(int addr=0; addr<=MAX_ADDR; addr++)
            {
                SPISlave spislave(bus, cs_M, cfg);
                SPITransaction spi(spislave);
                regset2[addr] = spi.read(addr);
                if(regset1[addr] == regset2[addr])
                {
                    printf("0x%02X-->0x%02X\n", addr,regset2[addr]);
                }
                else
                {
                    printf("0x%02X-->0x%02X  <--\n", addr,regset2[addr]);
                }
            }
        }
        else
            LED.low(); 
    }*/

    LSM9DS1_M lsm9ds1(
                    bus,
                    cs_M,
                    LSM9DS1_M::MagFSR::FS_12,
                    LSM9DS1_M::ODR::ODR_40
                    );

    while(!lsm9ds1.init());
    LED.high(); //init OK

    Thread::sleep(500);
    printf("x;y;z\n");
    long long first_tick = getTick();  
    for(;;)
    {   
        long long last_tick = getTick(); 
        lsm9ds1.onSimpleUpdate();
        mdata = *(lsm9ds1.compassDataPtr());
        printf("%d;%.3f;%.3f;%.3f\n",
                (int)(last_tick - first_tick), 
                mdata.getX(), mdata.getY(), mdata.getZ()
                );
    }
    


}
