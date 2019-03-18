/* LM75B Driver
 *
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Alessio Galluccio
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


//NOTE:
//fai array di due byte per temperatura

#ifndef LM75B_H
#define LM75B_H

#include "Sensor.h"

template <typename BusType> 
class LM75B: public TemperatureSensor
{
    public:
        // TO DO
        // @param slaveAddr     address of the sensor you want to use

        LM75B(uint8_t slaveAddr)
        {
            mLastTemp = 0;
            temp_array = {0, 0};
            slave_addr = slaveAddr;
            init();
        }

        bool init() 
        {
            BusType::write(slaveAddr, REG_CONF, &CONF_NORM, 1);
            return true;
        }

        bool onSimpleUpdate()
        {
            mLastTemp = getTemp();
            return true;
        }

        // TO DO controllare il segno
        float getTemp()
        {
            uint16_t temp;
            BusType::read(slaveAddr, REG_TEMP, temp_array, 2);
            temp = temp_array[0];
            
            if(temp >= 0) 
            {
                temp << 8;
                temp = temp + temp_array[1];
                temp = temp >> 5;
                return float(temp)*0.125;
            }
            else
            {
                temp << 8;
                temp = temp + temp_array[1];
                temp = temp >> 5;
                return float(-temp)*0.125;
            }
            
            
        }



    private:
    
        // slave address first sensor is 0b1001000
        // slave address second sensor is 0b1001001
        // (slave address third sensor is 0b1001010) 
        static constexpr uint8_t slave_addr; 

        uint8_t temp_array[2];

        enum Registers
        {
            REG_CONF = 0x1H;   // R/W
            REG_TEMP = 0x0H;   // R only
            REG_TOS = 0x3H;    // R/W
            REG_THYST = 0x2H;  // R/W
        };

        //TO DO controllare valore della configurazione normale
        enum ConfCommands
        {
            CONF_NORM = 0x00;
        }


};






#endif

