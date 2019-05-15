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

#ifndef LM75B_H
#define LM75B_H

#include "Sensor.h"
#include "math/Stats.h"

template <typename BusType> 
class LM75B: public TemperatureSensor
{
    public:
        /**
         * @param slaveAddr     address of the sensor you want to use
         */
        LM75B(uint8_t slaveAddr) : slave_addr(slaveAddr)
        {
            mLastTemp = 0.0f;
            //init();
        }

        /**
         * Initialize the sensor, writing the conf register and then reading it back.
         * @return   whether the configuration was done correctly
         */
        bool init() override
        {
            uint8_t conf = static_cast<uint8_t>(CONF_NORM);
            BusType::write(slave_addr, REG_CONF, &conf, sizeof(uint8_t));

            // Check if the register was set correctly
            BusType::read(slave_addr, REG_CONF, &conf, sizeof(uint8_t));
            bool conf_ok = (conf == static_cast<uint8_t>(CONF_NORM));

            if(conf_ok)
            {
                conf_ok = selfTest();
            }

            return conf_ok;
        }

        /**
         * Performed tests:
         * - Read THYST and TOS known registers
         * - Collect a bunch of samples and calculate stddev
         *
         * @return true if all tests passed
         */
        bool selfTest() override
        {
            //TEST 1: reading default value of THYST register, which is known
            uint8_t value_thyst;
            BusType::read(slave_addr, REG_THYST, &value_thyst, sizeof(uint8_t));

            if(value_thyst != THYST_DEFAULT_VAL)
            {
                TRACE("[LM75B] Error: reading wrong value of THYST register in LM75B\n");
                return false;
            }

            //TEST 2: reading default value of TOS register, which is known
            uint8_t value_tos;
            BusType::read(slave_addr, REG_TOS, &value_tos, sizeof(uint8_t));

            if(value_tos != TOS_DEFAULT_VALUE)
            {
                TRACE("[LM75B] Error: reading wrong value of THYST register in LM75B\n");
                return false;
            }

            //TEST 3: standard deviation of temperature value

            // TODO: check actual MAX_VALUE of temp std dev
            /*
            float stdev;
            Stats calc_stats;

            // Get a bunch of samples
            for(uint8_t i = 0; i < NUM_TEST_SAMPLES; i++) 
            {
                onSimpleUpdate();

                //temperature can't be out of range of sensor
                if(mLastTemp > -125.0 && mLastTemp < 125.0)
                {
                    calc_stats.add(mLastTemp);
                }
                else
                {
                    TRACE("[LM75B] Error: Temperature out of range\n");
                    return false;
                }
            }

            // Calculate standard dev
            stdev = calc_stats.getStats().stdev;
            if(stdev < MAX_STDEV_VALUE)
            {
                TRACE("[LM75B] Error: Standard deviation of temparature is out of range in LM75B");
                //return false;
            }
            */

            return true;
        }

        /**
         * Update temperature value.
         * @return always true
         */
        bool onSimpleUpdate() override
        {
            mLastTemp = updateTemp();
            return true;
        }

        /**
         * Unsynchronized getter of the last temperature value.
         * @return last temperature value
         */
        float getTemp()
        {
            return mLastTemp;
        }

    private:
        static const uint8_t THYST_DEFAULT_VAL = 75;
        static const uint8_t TOS_DEFAULT_VALUE = 80;
        static const uint8_t NUM_TEST_SAMPLES  = 10;
        static constexpr float MAX_STDEV_VALUE = 100.0f;

        const uint8_t slave_addr;       

        enum Registers
        {
            REG_CONF  = 0x01,  // R/W
            REG_TEMP  = 0x00,  // R only
            REG_TOS   = 0x03,  // R/W
            REG_THYST = 0x02   // R/W
        };

        enum ConfCommands
        {
            CONF_NORM = 0x00
        };

        // TODO controllare il segno
        float updateTemp()
        {
            uint8_t temp_array[2];
            uint16_t temp;

            BusType::read(slave_addr, REG_TEMP, temp_array, sizeof(uint16_t));
            // Switch endianness
            temp = (uint16_t) ((temp_array[0] << 8) | temp_array[1]);
            // Last 5 bits have no meaning
            temp >>= 5;

            return float(temp)*0.125;
        }

};

#endif