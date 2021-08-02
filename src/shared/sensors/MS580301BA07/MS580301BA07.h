/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Davide Benini, Matteo Piazzolla, Alain Carlucci
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "../Sensor.h"
#include "Debug.h"
#include "MS580301BA07Data.h"
#include "TimestampTimer.h"
#include "drivers/spi/SPIDriver.h"
#include <cstring>

enum MS5803Errors : uint8_t
{
    RESET_TIMEOUT = SensorErrors::END_OF_BASE_ERRORS
};

class MS580301BA07 : public Sensor<MS5803Data>
{

public:
    /* Class constructor. Reset lastPressure and lastTemperature */
    MS580301BA07(SPIBusInterface& bus, GpioPin cs, uint16_t temp_div = 1)
        : MS580301BA07(bus, cs, SPIBusConfig{}, temp_div)
    {
        spi_ms5803.config.clock_div = SPIClockDivider::DIV128;
    }

    MS580301BA07(SPIBusInterface& bus, GpioPin cs, SPIBusConfig config, uint16_t temp_div = 1)
        : spi_ms5803(bus, cs, config), mLastTemp(0.0), mLastPressure(0.0), temp_divider(temp_div)
    {
        memset(&cd, 0, sizeof(calibration_data));
    }

    ~MS580301BA07() {}

    bool init() override
    {
        SPITransaction spi{spi_ms5803};

        int timeout = 10;
        do
        {
            spi.read(RESET_DEV);

            miosix::Thread::sleep(3);

            cd.sens = readReg(spi, PROM_READ_MASK | PROM_SENS_MASK);
            if (cd.sens == 0)
            {
                miosix::Thread::sleep(1);
                TRACE("Could not read cd.sens\n");
            }
        } while (cd.sens == 0 && --timeout > 0);

        if (timeout <= 0)
        {
            last_error = MS5803Errors::RESET_TIMEOUT;
            return false;
        }

        cd.off      = readReg(spi, PROM_READ_MASK | PROM_OFF_MASK);
        cd.tcs      = readReg(spi, PROM_READ_MASK | PROM_TCS_MASK);
        cd.tco      = readReg(spi, PROM_READ_MASK | PROM_TCO_MASK);
        cd.tref     = readReg(spi, PROM_READ_MASK | PROM_TREF_MASK);
        cd.tempsens = readReg(spi, PROM_READ_MASK | PROM_TEMPSENS_MASK);

        TRACE("[MS5803] Init : off=%d, tcs=%d, tco=%d, tref=%d, tsens=%d\n",
              (int)cd.off, (int)cd.tcs, (int)cd.tco, (int)cd.tref,
              (int)cd.tempsens);

        mStatus = 0;

        return true;
    }

    bool selfTest() override { return true; }

    /**
     * Implements a state machines composed of 3 states:
     * 1. Command pressure sample
     * 2. Read Pressure sample & command temperature sample
     * 3. Read temperature sample & command pressure sample
     *
     * After the first call to sample() (state 1), the machine
     * transitions between states 2 and 3: The effectoive sampling rate is half
     * the rate at which this function is called.
     * Example: call sample() at 100 Hz -> Pressure & Temperature sample
     * Rate = 50 Hz
     */
    MS5803Data sampleImpl() override
    {
        SPITransaction spi{spi_ms5803};  // Begin an SPI transaction

        uint8_t rcvbuf[3];
        uint32_t temperature = 0;

        MS5803Data ms5803_data = last_sample;

        switch (mStatus)
        {
            case STATE_INIT:
                spi.write(CONVERT_D1_4096);
                mStatus = STATE_SAMPLED_PRESSURE;
                break;
            case STATE_SAMPLED_PRESSURE:
                spi.read(ADC_READ, rcvbuf, 3, false);
                mInternalPressure = rcvbuf[2] | ((uint32_t)rcvbuf[1] << 8) |
                                    ((uint32_t)rcvbuf[0] << 16);

                spi.write(CONVERT_D2_4096);  // Begin temperature sampling

                temp_counter++;
                // move to temperature sampling every temp_divider iterations
                if (temp_counter == temp_divider)
                {
                    temp_counter = 0;
                    mStatus = STATE_SAMPLED_TEMPERATURE;
                }
                break;

            case STATE_SAMPLED_TEMPERATURE:
                spi.read(ADC_READ, rcvbuf, 3, false);

                // TODO use swapBytes
                temperature = (uint32_t)rcvbuf[2] | ((uint32_t)rcvbuf[1] << 8) |
                              ((uint32_t)rcvbuf[0] << 16);

                ms5803_data = updateData(mInternalPressure, temperature);
                spi.write(CONVERT_D1_4096);  // Begin pressure sampling
                mStatus = STATE_SAMPLED_PRESSURE;
                break;

            default:
                ms5803_data = last_sample;
        }

        return ms5803_data;
    }

    enum FSM_State
    {
        STATE_INIT                = 0,
        STATE_SAMPLED_PRESSURE    = 1,
        STATE_SAMPLED_TEMPERATURE = 2
    };

    uint8_t getState() { return mStatus; }

private:
    SPISlave spi_ms5803;

    static constexpr uint8_t TIMEOUT = 5;
    uint8_t mStatus;
    uint32_t mInternalPressure;
    float mLastTemp;
    float mLastPressure;
    uint16_t temp_divider;
    uint16_t temp_counter = 0;

    MS5803Errors last_error;

    MS5803Data updateData(uint32_t pressure, uint32_t temperature)
    {
        int32_t dt   = temperature - (((uint32_t)cd.tref) << 8);
        int32_t temp = 2000 + (((uint64_t)dt * cd.tempsens) >> 23);

        int64_t offs = ((int64_t)cd.off << 16) + (((int64_t)cd.tco * dt) >> 7);
        int64_t sens = ((int64_t)cd.sens << 15) + (((int64_t)cd.tcs * dt) >> 8);

        int64_t t2 = 0, off2 = 0, sens2 = 0;

        // Second order temperature compensation
        if (temp < 2000)
        {
            t2    = (((int64_t)dt) * dt) >> 31;
            off2  = 3 * (temp - 2000) * (temp - 2000);
            sens2 = (7 * (temp - 2000) * (temp - 2000)) >> 3;

            if (temp < -1500)
            {
                sens2 = sens2 + 2 * (temp + 1500) * (temp + 1500);
            }
        }
        else if (temp >= 4500)
        {
            sens2 = sens2 - (((temp - 4500) * (temp - 4500)) >> 3);
        }

        temp = temp - t2;
        offs = offs - off2;
        sens = sens - sens2;

        int64_t ttemp = ((int64_t)pressure) * sens;
        int32_t pres  = ((ttemp >> 21) - offs) >> 15;

        mLastTemp     = temp / 100.0f;
        mLastPressure = pres;

        return MS5803Data(TimestampTimer::getTimestamp(), mLastPressure,
                          mLastTemp);
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

    uint16_t readReg(SPITransaction& spi, uint8_t reg)
    {
        uint8_t rcv[2];
        spi.read(reg, rcv, 2);
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
