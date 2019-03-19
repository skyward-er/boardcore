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

enum class SlaveAddress: uint8_t
{
    ADDR_1 = 0x48,
    ADDR_2 = 0x49,
    ADDR_3 = 0x50
};

template <typename BusType> 
class LM75B: public TemperatureSensor
{
    public:
        // TO DO
        // @param slaveAddr     address of the sensor you want to use
        LM75B(SlaveAddress slaveAddr) : slave_addr(static_cast<uint8_t>(slaveAddr))
        {
            mLastTemp = 0;
            init();
        }

        bool selfTest() override
        {
            // TODO: self test me!!
            return true;
        }

        bool init() override
        {
            uint8_t conf = static_cast<uint8_t>(CONF_NORM);
            BusType::write(slave_addr, REG_CONF, &conf, sizeof(uint8_t));
            return true;
        }

        bool onSimpleUpdate() override
        {
            mLastTemp = updateTemp();
            return true;
        }

        float getTemp()
        {
            return mLastTemp;
        }

    private:
        const uint8_t slave_addr; 

        uint8_t temp_array[2] = {0, 0};

        enum Registers
        {
            REG_CONF = 0x01,   // R/W
            REG_TEMP = 0x00,   // R only
            REG_TOS = 0x03,    // R/W
            REG_THYST = 0x02   // R/W
        };

        //TODO controllare valore della configurazione normale
        enum ConfCommands
        {
            CONF_NORM = 0x00
        };

        // TODO con che ordine vengono letti i byte?
        // TODO controllare il segno
        float updateTemp()
        {
            uint16_t temp;
            BusType::read(slave_addr, REG_TEMP, temp_array, sizeof(uint16_t));
            temp = temp_array[0];

            // MSB is the sign: 0->positive, 1->negative
            static const uint8_t msb_mask = 0x80;
            bool is_positive = ((temp_array[0] & msb_mask) == 0);

            if(is_positive) 
            {
                temp <<= 8;
                temp = temp | temp_array[1];
                temp = temp >> 5;
                return float(temp)*0.125;
            }
            else
            {
                temp <<= 8;
                temp = temp | temp_array[1];
                temp = temp >> 5;
                return float(-temp)*0.125;
            }
        }


};






#endif

