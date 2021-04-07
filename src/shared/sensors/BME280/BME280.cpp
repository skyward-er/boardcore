/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#include "BME280.h"

#include "Debug.h"
#include "TimestampTimer.h"

BME280::BME280(SPISlave spiSlave_) : spiSlave(spiSlave_) {}

bool BME280::init()
{
    // Check if already initialized
    if (initialized)
    {
        TRACE("[BME280] Already initialized\n");

        last_error = SensorErrors::ALREADY_INIT;

        return false;
    }

    // Check WHO AM I
    if (!checkWhoAmI())
    {
        last_error = SensorErrors::INVALID_WHOAMI;

        return false;
    }

    // Load compensation parameters
    loadCompensationParameters();

    TRACE("dig_T1: %d\t%X\n", compParams.bits.dig_T1, compParams.bits.dig_T1);
    TRACE("dig_T2: %d\t%X\n", compParams.bits.dig_T2, compParams.bits.dig_T2);
    TRACE("dig_T3: %d\t%X\n", compParams.bits.dig_T3, compParams.bits.dig_T3);
    TRACE("dig_P1: %d\t%X\n", compParams.bits.dig_P1, compParams.bits.dig_P1);
    TRACE("dig_P2: %d\t%X\n", compParams.bits.dig_P2, compParams.bits.dig_P2);
    TRACE("dig_P3: %d\t%X\n", compParams.bits.dig_P3, compParams.bits.dig_P3);
    TRACE("dig_P4: %d\t%X\n", compParams.bits.dig_P4, compParams.bits.dig_P4);
    TRACE("dig_P5: %d\t%X\n", compParams.bits.dig_P5, compParams.bits.dig_P5);
    TRACE("dig_P6: %d\t%X\n", compParams.bits.dig_P6, compParams.bits.dig_P6);
    TRACE("dig_P7: %d\t%X\n", compParams.bits.dig_P7, compParams.bits.dig_P7);
    TRACE("dig_P8: %d\t%X\n", compParams.bits.dig_P8, compParams.bits.dig_P8);
    TRACE("dig_P9: %d\t%X\n", compParams.bits.dig_P9, compParams.bits.dig_P9);
    TRACE("dig_H1: %d\t%X\n", compParams.bits.dig_H1, compParams.bits.dig_H1);
    TRACE("dig_H2: %d\t%X\n", compParams.bits.dig_H2, compParams.bits.dig_H2);
    TRACE("dig_H3: %d\t%X\n", compParams.bits.dig_H3, compParams.bits.dig_H3);
    TRACE("dig_H4: %d\t%X\n", compParams.bits.dig_H4, compParams.bits.dig_H4);
    TRACE("dig_H5: %d\t%X\n", compParams.bits.dig_H5, compParams.bits.dig_H5);
    TRACE("dig_H6: %d\t%X\n", compParams.bits.dig_H6, compParams.bits.dig_H6);

    // TODO: Read at least once the temperature for t_fine

    // Set the configuration
    setConfiguration();

    initialized = true;
    return true;
}

bool BME280::selfTest() { return checkWhoAmI(); }

BME280Data BME280::sampleImpl()
{
    BME280RawData rawData;
    BME280Data data;

    // TODO: implement selective read!

    // Burst read pressure, temperature and humidity
    {
        SPITransaction transaction(spiSlave);

        transaction.read(REG_CALIB_0, (uint8_t *)&rawData, 25);
    }

    // Compensate temperature
    t_fine              = getFineTemperature(rawData.bits.temperature);
    data.temp_timestamp = TimestampTimer::getTimestamp();
    data.temp           = compensateTemperature(t_fine);

    // Compensate pressure
    data.press_timestamp = TimestampTimer::getTimestamp();
    data.press           = compensatePressure(rawData.bits.pressure);

    // Compensate humidity
    data.humid_timestamp = TimestampTimer::getTimestamp();
    data.humid           = compensateHumidity(rawData.bits.pressure);
}

bool BME280::checkWhoAmI()
{
    SPITransaction transaction(spiSlave);

    uint8_t who_am_i_value = transaction.read(WHO_AM_I_REG);

    return who_am_i_value == WHO_AM_I_VAL;
}

void BME280::setConfiguration()
{
    SPITransaction transaction(spiSlave);

    for (uint8_t i = 0; i < 4; i++)
    {
        transaction.write(REG_CTRL_HUM + i, config.bytes_array[i]);
    }
}

void BME280::loadCompensationParameters()
{
    // Read first batch of compensation parameters
    {
        SPITransaction transaction(spiSlave);

        transaction.read(REG_CALIB_0, (uint8_t *)&compParams, 25);
    }

    // Reat second batch of compensation parameters
    {
        SPITransaction transaction(spiSlave);

        transaction.read(REG_CALIB_26, (uint8_t *)&compParams.bits.dig_H2, 7);
    }

    // Adjust unaligned data
    compParams.bytes_array[29] =
        (compParams.bytes_array[29] << 4) | (compParams.bytes_array[29] >> 4);
    compParams.bits.dig_H4 =
        (compParams.bits.dig_H4 << 4) | (compParams.bits.dig_H4 >> 8);
}

int32_t BME280::getFineTemperature(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)compParams.bits.dig_T1 << 1))) *
            ((int32_t)compParams.bits.dig_T2)) >>
           11;
    var2 = (((((adc_T >> 4) - ((int32_t)compParams.bits.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)compParams.bits.dig_T1))) >>
             12) *
            ((int32_t)compParams.bits.dig_T3)) >>
           14;
    return var1 + var2;
}

int32_t BME280::compensateTemperature(int32_t t_fine)
{
    return (t_fine * 5 + 128) >> 8;
}

uint32_t BME280::compensatePressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)compParams.bits.dig_P6;
    var2 = var2 + ((var1 * (int64_t)compParams.bits.dig_P5) << 17);
    var2 = var2 + (((int64_t)compParams.bits.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)compParams.bits.dig_P3) >> 8) +
           ((var1 * ((int64_t)compParams.bits.dig_P2) << 12));
    var1 =
        ((((int64_t)1) << 47) + var1) * ((int64_t)compParams.bits.dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0;  // avoid exception caused by division by zero
    }
    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)compParams.bits.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)compParams.bits.dig_P8) * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + (((int64_t)compParams.bits.dig_P7) << 4);
    return (uint32_t)p;
}

uint32_t BME280::compensateHumidity(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)768000));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)compParams.bits.dig_H4) << 20) -
                    (((int32_t)compParams.bits.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >>
                  15) *
                 (((((((v_x1_u32r * ((int32_t)compParams.bits.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)compParams.bits.dig_H3)) >> 11) +
                       ((int32_t)32768))) >>
                     10) +
                    ((int32_t)2097152)) *
                       ((int32_t)compParams.bits.dig_H2) +
                   8192) >>
                  14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)compParams.bits.dig_H1)) >>
                              4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}
