/* MS580301BA07 Driver
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Davide Benini, Matteo Michele Piazzolla, Alain Carlucci
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

#include <drivers/BusTemplate.h>
#include "../Sensor.h"
#include "MS580301BA07Data.h"

// TODO second order temperature compensation
template <class Bus>
class MS580301BA07 : public PressureSensor, public TemperatureSensor
{

public:
    /* Class constructor. Reset lastPressure and lastTemperature */
    MS580301BA07()
    {
        memset(&cd, 0, sizeof(calibration_data));
        mLastTemp     = 0;
        mLastPressure = 0;
    }

    ~MS580301BA07() {}

    bool init()
    {
        int timeout = 30;
        do
        {
            Bus::read(RESET_DEV);

            miosix::Thread::sleep(3);

            cd.sens = readReg(PROM_READ_MASK | PROM_SENS_MASK);
            if (cd.sens == 0)
                miosix::Thread::sleep(1);
        } while (cd.sens == 0 && --timeout > 0);

        if (timeout <= 0)
        {
            last_error = ERR_RESET_TIMEOUT;
            return false;
        }

        cd.off      = readReg(PROM_READ_MASK | PROM_OFF_MASK);
        cd.tcs      = readReg(PROM_READ_MASK | PROM_TCS_MASK);
        cd.tco      = readReg(PROM_READ_MASK | PROM_TCO_MASK);
        cd.tref     = readReg(PROM_READ_MASK | PROM_TREF_MASK);
        cd.tempsens = readReg(PROM_READ_MASK | PROM_TEMPSENS_MASK);

        mStatus = 0;

        return true;
    }

    bool selfTest() { return false; }

    /**
     * Implements a state machines composed of 3 states:
     * 1. Command pressure sample
     * 2. Read Pressure sample & command temperature sample
     * 3. Read temperature sample & command pressure sample
     *
     * After the first call to onSimpleUpdate() (state 1), the machine
     * transitions between states 2 and 3: The effectoive sampling rate is half
     * the rate at which this function is called.
     * Example: call onSimpleUpdate() at 100 Hz -> Pressure & Temperature sample
     * Rate = 50 Hz
     */
    bool onSimpleUpdate()
    {
        uint8_t rcvbuf[3];
        uint32_t temperature = 0;

        switch (mStatus)
        {
            case STATE_INIT:
                Bus::write(CONVERT_D1_4096);
                mStatus = STATE_SAMPLED_PRESSURE;
            case STATE_SAMPLED_PRESSURE:
                Bus::read_low(ADC_READ, rcvbuf, 3);
                mInternalPressure = rcvbuf[2] | ((uint32_t)rcvbuf[1] << 8) |
                                    ((uint32_t)rcvbuf[0] << 16);

                Bus::write(CONVERT_D2_4096);  // Begin temperature sampling
                mStatus = STATE_SAMPLED_TEMPERATURE;

                break;
            case STATE_SAMPLED_TEMPERATURE:
                Bus::read_low(ADC_READ, rcvbuf, 3);

                // TODO use swapBytes
                temperature = (uint32_t)rcvbuf[2] | ((uint32_t)rcvbuf[1] << 8) |
                              ((uint32_t)rcvbuf[0] << 16);

                updateData(mInternalPressure, temperature);
                Bus::write(CONVERT_D1_4096);  // Begin pressure sampling
                mStatus = STATE_SAMPLED_PRESSURE;

                break;
        }

        return true;
    }

    MS5803Data getData() { return dataStruct; }

    enum FSM_State
    {
        STATE_INIT                = 0,
        STATE_SAMPLED_PRESSURE    = 1,
        STATE_SAMPLED_TEMPERATURE = 2
    };

    uint8_t getState()
    {
        return mStatus;
    }

private:
    static constexpr uint8_t TIMEOUT = 5;
    uint8_t mStatus;
    uint32_t mInternalPressure;

    MS5803Data dataStruct;

    void updateData(uint32_t pressure, uint32_t temperature)
    {
        int32_t dt   = temperature - (cd.tref << 8);
        int32_t temp = 2000 + ((dt * cd.tempsens) >> 23);
        mLastTemp    = temp / 100.0f;

        int64_t offs = ((int64_t)cd.off << 16) + (((int64_t)cd.tco * dt) >> 7);
        int64_t senst =
            ((int64_t)cd.sens << 15) + (((int64_t)cd.tcs * dt) >> 8);

        int64_t ttemp = pressure * senst;
        int32_t pres  = ((ttemp >> 21) - offs) >> 15;
        mLastPressure = pres;

        dataStruct.raw_temp  = temperature;
        dataStruct.temp      = mLastTemp;
        dataStruct.raw_press = pressure;
        dataStruct.pressure  = mLastPressure;
        dataStruct.timestamp = miosix::getTick();
    }

    typedef struct
    {
        uint16_t sens;
        uint16_t off;
        uint16_t tcs;
        uint16_t tco;
        uint16_t tref;
        uint16_t tempsens;
    } calibration_data;

    calibration_data cd;

    uint16_t readReg(uint8_t reg)
    {
        uint8_t rcv[2];
        Bus::read(reg, rcv, 2);
        uint16_t data = (rcv[0] << 8) | rcv[1];
        return data;
    }

    // clang-format off
    enum eRegisters
    {
        RESET_DEV                 = 0x1E,

        CONVERT_D1_256            = 0x40,
        CONVERT_D1_512            = 0x42,
        CONVERT_D1_1024           = 0x44,
        CONVERT_D1_2048           = 0x46,
        CONVERT_D1_4096           = 0x48,

        CONVERT_D2_256            = 0x50,
        CONVERT_D2_512            = 0x52,
        CONVERT_D2_1024           = 0x54,
        CONVERT_D2_2048           = 0x56,
        CONVERT_D2_4096           = 0x58,

        ADC_READ                  = 0x00,

        PROM_READ_MASK            = 0xA0,
        PROM_SENS_MASK            = 0x02,
        PROM_OFF_MASK             = 0x04,
        PROM_TCS_MASK             = 0x06,
        PROM_TCO_MASK             = 0x08,
        PROM_TREF_MASK            = 0x0A,
        PROM_TEMPSENS_MASK        = 0x0C,
    };
    // clang-format on
};
