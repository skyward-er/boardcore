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
#include "math/Stats.h"

#define NUM_SAMPLES 10
#define MSB_BYTE 0
#define LSB_BYTE 1

enum class SlaveAddress: uint8_t
{
    ADDR_1 = 0x48,  //first TempSensor
    ADDR_2 = 0x49,  //second TempSensor
    ADDR_3 = 0x50   //third TempSensor (not in Rocksanne)
};

template <typename BusType> 
class LM75B: public TemperatureSensor
{
    public:
        // @param slaveAddr     address of the sensor you want to use
        LM75B(SlaveAddress slaveAddr) : slave_addr(static_cast<uint8_t>(slaveAddr))
        {
            mLastTemp = 7;
            init();
        }

         //TODO aggiungimi controllo varianza
        bool selfTest() override
        {
            float stdev;
            Stats calc_stats;

            for(int i = 0; i < NUM_SAMPLES; i++) 
            {
                onSimpleUpdate();
                sample[i] = mLastTemp;
            }
            
            for(int i = 0; i < NUM_SAMPLES; i++)
            {
                //temperature can't be out of range of sensor
                if(sample[i] < -125.0 && sample[i] > 125.0)
                {
                    return false;
                }

            }

            for(int i = 0; i < NUM_SAMPLES; i++)
            {
                calc_stats.add(sample[i]);
            }
            stdev = calc_stats.getStats().stdev;

            if(stdev < MAX_STDEV_VALUE)
            {
                return true;
            }
            else
            {
                return false;
            }
            
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

        //max Standard deviation value accepted for temperature samples 
        const float MAX_STDEV_VALUE = 100;

        float sample[NUM_SAMPLES];

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
            uint16_t temp, foodx, foosx;
            uint8_t temp_for_swap;
            BusType::read(slave_addr, REG_TEMP, temp_array, sizeof(uint16_t));


            TRACE("Read: %x %x\n", temp_array[0], temp_array[1]);

            //they must be swapped
            temp_for_swap = temp_array[0];
            temp_array[0] = temp_array[1];
            temp_array[1] = temp_for_swap;
             
            memcpy(&temp, temp_array, sizeof(uint16_t));
            TRACE("UINT16: %x\n", temp);
            foodx = temp >> 5;
            TRACE("SHIFTED 5 right: %x, FLOAT_VAL: %f\n", foodx, float(foodx)*0.125);
            foosx = temp << 5;
            TRACE("SHIFTED 5 left: %x, FLOAT_VAL: %f\n", foosx, float(foosx)*0.125);

            return float(foodx)*0.125;

            /*
            temp = temp_array[MSB_BYTE];
            // MSB is the sign: 0->positive, 1->negative
            static const uint8_t msb_mask = 0x80;
            bool is_positive = ((temp_array[MSB_BYTE] & msb_mask) == 0);
            TRACE("temp is before: %d \n", temp);
            //TRACE("bool is: %d\n", is_positive);
            if(is_positive) 
            {
                //TRACE("positive\n");
                temp <<= 8;
                TRACE("after <<8 : %d \n", temp);
                temp = temp | uint16_t(temp_array[LSB_BYTE]);
                TRACE("after |  : %d \n", temp);
                temp = temp >> 5;
                TRACE("after >>5 : %d \n", temp);
                return float(temp)*0.125;
            }
            else
            {
                TRACE("negative\n");
                temp <<= 8;
                temp = temp | uint16_t(temp_array[LSB_BYTE]);
                temp = temp >> 5;
                return -float(temp)*0.125;
            }*/
        }


};

#endif