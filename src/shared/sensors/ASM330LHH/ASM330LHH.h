/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Vincenzo Santomarco
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

#pragma once

#include "sensors/Sensor.h"
#include "Common.h"
#include "drivers/spi/SPIDriver.h"

using miosix::getTick;
using miosix::TICK_FREQ;
 
class ASM330LHH : public virtual Sensor
{
    public:
        ASM330LHH(SPIBusInterface& bus,
                    GpioPin chip_select) : 
                spi_slave(bus, chip_select, {}) 
        {
            spi_slave.config.clock_div = SPIClockDivider::DIV64;  // used to set the spi baud rate (maximum is 10 Mhz)
        }

         ASM330LHH(SPIBusInterface& bus,
                    GpioPin chip_select,
                    SPIBusConfig config) :
                spi_slave(bus, chip_select, config) {}

        ~ASM330LHH() {}
 
        bool init() override
        { 

            miosix::Thread::sleep(10);

            setup_device();

            if(!check_whoami()){
                TRACE("Whoami check failed!\n");
                return false;
            }
            return true; 
        }
 
        bool onSimpleUpdate() override
        {
            return true;
        }
 
        bool selfTest() override 
        {  
            // check that the sensor is properly working
            // e.g. check values tolerance over a set of samples
            //      or what the datasheet suggests
            return true; 
        }
        
        // not a Sensor class method: you should 
        // define methods to get the sampled data
        float getData() 
        { 
            return 0; 
        }
    
    private:
        /**
         * @brief Check that the WHO_AM_I register 
         *        contains the correct value.
         * 
         * @return boolean value indicating whether the value read
         *          from the WHO_AM_I register is correct or not
         */
        bool check_whoami() 
        {
            SPITransaction spi(spi_slave);
            
            uint8_t whoami = spi.read(WHO_AM_I_REG);

            TRACE("whoami: %u\n", whoami);

            return whoami == WHO_AM_I_DEFAULT_VALUE;
        }

        /**
         * @brief I2C_disable = 1 in CTRL4_C and
         *      DEVICE_CONF = 1 in CTRL9_XL for setting up
         *      the device for SPI
         */
        void setup_device() 
        {
            SPITransaction spi(spi_slave);

            // Read and update CTRL4_C
            uint8_t ctrl4_c_val = spi.read(CTRL4_C_REG);
            TRACE("ctrl4_c_val: %u\n", ctrl4_c_val);
            uint8_t i2c_disable_bit = 4;                    // 0b00000100
            ctrl4_c_val |= i2c_disable_bit;
            spi.write(CTRL4_C_REG, ctrl4_c_val);


            // Read and update CTRL9_XL
            uint8_t ctrl9_xl_val = spi.read(CTRL9_XL_REG);
            TRACE("ctrl9_xl_val: %u\n", ctrl9_xl_val);
            uint8_t device_conf = 2;                        // 0b00000010
            ctrl9_xl_val |= device_conf;
            spi.write(CTRL9_XL_REG, ctrl9_xl_val);

        }
    
    // Constant values
    private:
        const uint8_t WHO_AM_I_DEFAULT_VALUE = 0x6b;

    private:

        /**
         * @brief Registers' addresses definition.
         */
        enum REG {
            WHO_AM_I_REG = 0x0f,
            CTRL4_C_REG = 0x13,
            CTRL9_XL_REG = 0x18,
        };

    private:
        SPISlave spi_slave;

};