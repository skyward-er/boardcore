/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include <stdio.h>

#include "BME280Data.h"
#include "sensors/Sensor.h"
#include "debug.h"

template <class Bus>
class BME280 : public TemperatureSensor, public PressureSensor
{
public:
    BME280() {}
    virtual ~BME280() {}

    std::string printData() { return ""; }

    bool init() override
    {
        if (!initialized)
        {
            Bus::init();

            uint8_t who_am_i = Bus::read(REG_WHO_AM_I);
            if (who_am_i != ID)
            {
                last_error = ERR_NOT_ME;
                TRACE("BME280 not found: who am i read 0x%02X\n", who_am_i);
                return false;
            }

            // T standby = 0.5 ms, Filter: 8x ("& 0x7F" to set bit 7 to 0)
            Bus::write(REG_CONFIG & 0x7F, 0x0C);

            uint8_t config_val = Bus::read(REG_CONFIG);
            if (config_val != 0x0C)
            {
                TRACE("Wrong value in config register: 0x%02X\n", config_val);
                return false;
            }

            // Disable humidity reading
            Bus::write(REG_CTRL_HUM & 0x7F, 0x00);

            uint8_t hum_val = Bus::read(REG_CTRL_HUM);
            if (hum_val != 0x00)
            {
                TRACE("Wrong value in hum register: 0x%02X\n", hum_val);
                last_error = ERR_BUS_FAULT;
                return false;
            }

            // Pressure oversampling x4, temp oversampling x1, normal mode.
            Bus::write(REG_CTRL_MEAS & 0x7F, (uint8_t)0x2F);

            uint8_t meas_val = Bus::read(REG_CTRL_MEAS);
            if (meas_val != 0x2F)
            {
                TRACE("Wrong value in meas register: 0x%02X\n", meas_val);
                last_error = ERR_BUS_FAULT;
                return false;
            }

            loadCompensationParams();

            initialized = true;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool selfTest() override { return true; }

    void printCompensationParams()
    {
        if (initialized)
        {
            printf("Compensation parameters: \n");

            printf("dig_T1: 0x%04X\n", comp_params.dig_T1);
            printf("dig_T2: 0x%04X\n", comp_params.dig_T2);
            printf("dig_T3: 0x%04X\n", comp_params.dig_T3);

            printf("dig_P1: 0x%04X\n", comp_params.dig_P1);
            printf("dig_P2: 0x%04X\n", comp_params.dig_P2);
            printf("dig_P3: 0x%04X\n", comp_params.dig_P3);
            printf("dig_P4: 0x%04X\n", comp_params.dig_P4);
            printf("dig_P5: 0x%04X\n", comp_params.dig_P5);
            printf("dig_P6: 0x%04X\n", comp_params.dig_P6);
            printf("dig_P7: 0x%04X\n", comp_params.dig_P7);
            printf("dig_P8: 0x%04X\n", comp_params.dig_P8);
            printf("dig_P9: 0x%04X\n", comp_params.dig_P9);
        }
    }

    bool onSimpleUpdate() override
    {
        if (initialized)
        {
            // Burst read temperature and pressure registers
            uint8_t buf[6];
            Bus::read(REG_PRESS_MSB, buf, 6);

            uint32_t press = 0;
            press |= ((uint32_t)buf[0]) << 12;
            press |= ((uint32_t)buf[1]) << 4;
            press |= ((uint32_t)buf[2]) >> 4;

            uint32_t temp = 0;
            temp |= ((uint32_t)buf[3]) << 12;
            temp |= ((uint32_t)buf[4]) << 4;
            temp |= ((uint32_t)buf[5]) >> 4;

            data.raw_pressure    = press;
            data.raw_temperature = temp;

            // Compensate values
            int32_t t_fine = getFineTemperature(data.raw_temperature);

            data.temperature = compensateTemperature(t_fine) / 100.0f;
            data.pressure =
                compensatePressure(data.raw_pressure, t_fine) / 256.0f;

            data.timestamp = miosix::getTick();

            mLastPressure = data.pressure;
            mLastTemp     = data.temperature;

            /*printf(
                "Raw P:\t%d,\tRaw T:\t%d\nComp P:\t%d,\tComp T:\t%d,\tFine T: "
                "%d\n",
                (int)press, (int)temp, (int)(pressure_value / 256),
                (int)temp_value, (int)t_fine);*/
            return true;
        }
        return false;
    }

    const uint32_t* rawPressureDataPtr() { return &data.raw_pressure; }

    const uint32_t* rawTempDataPtr() { return &data.raw_temperature; }

    const BME280Data* getDataPtr() { return &data; }

    BME280Data data;

private:
    void loadCompensationParams()
    {
        uint8_t buf[24];
        Bus::read(REG_CALIB_T, buf, 24);

        comp_params.dig_T1 = toUInt16(&buf[0]);
        comp_params.dig_T2 = toInt16(&buf[2]);
        comp_params.dig_T3 = toInt16(&buf[4]);

        comp_params.dig_P1 = toUInt16(&buf[6]);
        comp_params.dig_P2 = toInt16(&buf[8]);
        comp_params.dig_P3 = toInt16(&buf[10]);
        comp_params.dig_P4 = toInt16(&buf[12]);
        comp_params.dig_P5 = toInt16(&buf[14]);
        comp_params.dig_P6 = toInt16(&buf[16]);
        comp_params.dig_P7 = toInt16(&buf[18]);
        comp_params.dig_P8 = toInt16(&buf[20]);
        comp_params.dig_P9 = toInt16(&buf[22]);
    }

    /**
     * Returns the temperature in degC.
     * Refer to the data sheet for a description of this method.
     */
    int32_t getFineTemperature(uint32_t raw_T)
    {
        int32_t adc_T = static_cast<int32_t>(raw_T);
        int32_t var1, var2;
        var1 = ((((adc_T / 8) - ((int32_t)comp_params.dig_T1 * 2))) *
                ((int32_t)comp_params.dig_T2)) >>
               11;
        var2 = (((((adc_T / 16) - ((int32_t)comp_params.dig_T1)) *
                  ((adc_T / 16) - ((int32_t)comp_params.dig_T1))) /
                 4096) *
                ((int32_t)comp_params.dig_T3)) /
               16384;

        return var1 + var2;
    }

    int32_t compensateTemperature(int32_t fine_temperature)
    {
        return (fine_temperature * 5 + 128) / 256;
    }

    uint32_t compensatePressure(uint32_t adc_P, int32_t fine_temperature)
    {
        int64_t var1, var2, p;
        var1 = ((int64_t)fine_temperature) - 128000;

        var2 = var1 * var1 * (int64_t)comp_params.dig_P6;
        var2 = var2 + ((var1 * (int64_t)comp_params.dig_P5) * 131072);
        var2 = var2 + (((int64_t)comp_params.dig_P4) * 34359738368);

        var1 = ((var1 * var1 * (int64_t)comp_params.dig_P3) / 256) +
               ((var1 * ((int64_t)comp_params.dig_P2) * 4096));

        var1 = (((int64_t)1) * 140737488355328 + var1) *
               ((int64_t)comp_params.dig_P1) / 8589934592;

        if (var1 == 0)
        {
            return 0;  // avoid exception caused by division by zero
        }

        p = 1048576 - (int32_t)adc_P;
        p = (((p * 2147483648) - var2) * 3125) / var1;

        var1 = (((int64_t)comp_params.dig_P9) * (p / 8192) * (p / 8192)) /
               33554432;

        var2 = (((int64_t)comp_params.dig_P8) * p) / 524288;

        p = ((p + var1 + var2) / 256) + (((int64_t)comp_params.dig_P7) * 16);

        // p = ((var3 / 2) * 100) / 128;
        return (uint32_t)p;
    }

    /**
     * Converts two bytes into an unsigned int 16
     * @param buf: buffer containing at least two bytes
     */
    uint16_t toUInt16(uint8_t* buf) { return buf[0] | buf[1] << 8; }

    /**
     * Converts two bytes into a signed int 16
     * @param buf: buffer containing at least two bytes
     */
    int16_t toInt16(uint8_t* buf)
    {
        return static_cast<int16_t>(buf[0] | buf[1] << 8);
    }

    enum Registers : uint8_t
    {
        REG_WHO_AM_I  = 0xD0,
        REG_RESET     = 0xE0,
        REG_CTRL_HUM  = 0xF2,
        REG_STATUS    = 0xF3,
        REG_CTRL_MEAS = 0xF4,
        REG_CONFIG    = 0xF5,

        REG_PRESS_MSB  = 0xF7,
        REG_PRESS_LSB  = 0xF8,
        REG_PRESS_XLSB = 0xF9,

        REG_TEMP_MSB  = 0xFA,
        REG_TEMP_LSB  = 0xFB,
        REG_TEMP_XLSB = 0xFC,

        REG_HUM_MSB = 0xFD,
        REG_HUM_LSB = 0xFE,

        REG_CALIB_T = 0x88,
        REG_CALUB_P = 0x8E
    };

    /**
         * Compensation parameters to convert ADC values to real temperature and
         * pressure
         */
    struct CompensationParams
    {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;

        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
    };

    bool initialized = false;

    const uint8_t ID         = 0x60;
    const uint8_t RESET_WORD = 0xB6;

    CompensationParams comp_params;
};
