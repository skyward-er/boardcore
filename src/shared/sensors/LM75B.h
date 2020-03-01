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

// TODO sistemare la codifica di {0x80, 0x00}, che
// per qualche ragione, diventa positiva

#ifndef LM75B_H
#define LM75B_H

#include "Sensor.h"
#include "math/Stats.h"

template <typename BusType>
class LM75B : public TemperatureSensor
{
public:
    // @param slaveAddr     address of the sensor you want to use
    LM75B(uint8_t slaveAddr) : slave_addr(static_cast<uint8_t>(slaveAddr))
    {
        mLastTemp = 7;
    }

    bool selfTest() override
    {
        bool flag_self_test = 1;  // 1 if test are passed, 0 if not

        // TEST: reading default value of THYST register
        uint8_t value_thyst;
        BusType::read(slave_addr, REG_THYST, &value_thyst, sizeof(uint8_t));
        if (value_thyst == default_value_thyst)
        {
            TRACE("\nReading correct value of THYST register in LM75B");
        }
        else
        {
            TRACE("\nWarning: reading wrong value of THYST register in LM75B");
            flag_self_test = 0;
        }

        // TEST: reading default value of TOS register
        uint8_t value_tos;
        BusType::read(slave_addr, REG_TOS, &value_tos, sizeof(uint8_t));
        if (value_tos == default_value_tos)
        {
            TRACE("\nReading correct value of TOS register in LM75B");
        }
        else
        {
            TRACE("\nWarning: reading wrong value of TOS register in LM75B");
            flag_self_test = 0;
        }

        // TEST: standard deviation of temperature value
        float stdev;
        Stats calc_stats;

        for (int i = 0; i < SELF_TEST_NUM_SAMPLES; i++)
        {
            onSimpleUpdate();
            sample[i] = mLastTemp;
        }

        for (int i = 0; i < SELF_TEST_NUM_SAMPLES; i++)
        {
            // temperature can't be out of range of sensor
            if (sample[i] < -125.0 && sample[i] > 125.0)
            {
                return false;
            }
        }

        for (int i = 0; i < SELF_TEST_NUM_SAMPLES; i++)
        {
            calc_stats.add(sample[i]);
        }
        stdev = calc_stats.getStats().stdev;

        if (stdev < MAX_STDEV_VALUE)
        {
            TRACE("\nStandard deviation of temperature is correct in LM75B");
        }
        else
        {
            TRACE(
                "\nWarning: Standard deviation of temparature is out of range "
                "in LM75B");
            flag_self_test = 0;
        }

        if (flag_self_test == 1)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    bool init() override
    {
        uint8_t conf = static_cast<uint8_t>(CONF_NORM);
        BusType::write(slave_addr, REG_CONF, &conf, sizeof(uint8_t));

        // Check if the register was set correctly
        BusType::read(slave_addr, REG_CONF, &conf, sizeof(uint8_t));
        bool conf_ok = (conf == static_cast<uint8_t>(CONF_NORM));

        return conf_ok;
    }

    bool onSimpleUpdate() override
    {
        mLastTemp = updateTemp();
        return true;
    }

    float getTemp() { return mLastTemp; }

    enum SlaveAddress : uint8_t
    {
        ADDR_1 = 0x48,  // first TempSensor
        ADDR_2 = 0x49,  // second TempSensor
        ADDR_3 = 0x50   // third TempSensor (not in Rocksanne)
    };

private:
    static constexpr int SELF_TEST_NUM_SAMPLES = 10;
    const uint8_t slave_addr;
    uint8_t temp_array[2] = {0, 0};

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

    // for testing
    const uint8_t default_value_thyst = 75;
    const uint8_t default_value_tos   = 80;
    const float MAX_STDEV_VALUE =
        100;  // max Standard deviation value accepted for temperature samples
    float sample[SELF_TEST_NUM_SAMPLES];

    float updateTemp()
    {
        BusType::read(slave_addr, REG_TEMP, temp_array, sizeof(uint16_t));
        // TRACE("Read: %x %x\n", temp_array[0], temp_array[1]);
        return computeTemp(temp_array);
    }

    // the temperature is saved in two bytes of the temp register
    // they must be swapped, concatenated and shifted right by 5 bits.
    // The resulting value must be multiplicated by 0.125
    float computeTemp(uint8_t *temp_array)
    {
        bool isNegative = false;

        // if most significant bit is 1, temperature is negative
        constexpr int MS_BIT_VALUE = 0x80;
        if (temp_array[0] >= MS_BIT_VALUE)
        {
            isNegative = true;
        }

        // they must be swapped, because the software reads the LSByte before
        // MSByte
        uint8_t temp_for_swap;
        temp_for_swap = temp_array[0];
        temp_array[0] = temp_array[1];
        temp_array[1] = temp_for_swap;

        int16_t temp;

        // TRACE("After swap: %x %x\n", temp_array[0], temp_array[1]);
        memcpy(&temp, temp_array, sizeof(int16_t));
        // TRACE("UINT16: %x\n", temp);

        if (isNegative)
        {
            // Two's complement
            temp = ~(temp);
            temp += 1;
        }

        // the five least significant bits must be eliminated
        uint16_t rightShift;
        constexpr int16_t RIGHT_SHIFT_5 = 0x20;

        rightShift = temp / RIGHT_SHIFT_5;
        // TRACE("After right shift: %x, %d\n", rightShift, rightShift);
        if (isNegative)
        {
            // TRACE("negative value returned");
            return -float(int16_t(rightShift)) * 0.125;
        }
        else
        {
            return float(int16_t(rightShift)) * 0.125;
        }
    }
};

#endif