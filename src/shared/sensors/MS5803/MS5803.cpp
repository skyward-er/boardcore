
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "MS5803.h"

#include "TimestampTimer.h"

namespace Boardcore
{

MS5803::MS5803(SPISlave spiSlave_, uint16_t temperatureDivider_)
    : spiSlave(spiSlave_), temperatureDivider(temperatureDivider_)
{
}

MS5803::MS5803(SPIBusInterface& spiBus_, miosix::GpioPin cs_,
               SPIBusConfig spiConfig_, uint16_t temperatureDivider_)
    : spiSlave(spiBus_, cs_, spiConfig_),
      temperatureDivider(temperatureDivider_)
{
}

bool MS5803::init()
{
    SPITransaction transaction{spiSlave};

    // Read calibration data
    calibrationData.sens =
        readReg(transaction, REG_PROM_READ_MASK | REG_PROM_SENS_MASK);
    calibrationData.off =
        readReg(transaction, REG_PROM_READ_MASK | REG_PROM_OFF_MASK);
    calibrationData.tcs =
        readReg(transaction, REG_PROM_READ_MASK | REG_PROM_TCS_MASK);
    calibrationData.tco =
        readReg(transaction, REG_PROM_READ_MASK | REG_PROM_TCO_MASK);
    calibrationData.tref =
        readReg(transaction, REG_PROM_READ_MASK | REG_PROM_TREF_MASK);
    calibrationData.tempsens =
        readReg(transaction, REG_PROM_READ_MASK | REG_PROM_TEMPSENS_MASK);

    LOG_INFO(logger, "Init: off={}, tcs={}, tco={}, tref={}, tsens={}",
             calibrationData.off, calibrationData.tcs, calibrationData.tco,
             calibrationData.tref, calibrationData.tempsens);

    return true;
}

bool MS5803::selfTest() { return true; }

MS5803Data MS5803::sampleImpl()
{
    SPITransaction transaction{spiSlave};

    uint8_t buffer[3];

    switch (deviceState)
    {
        case STATE_INIT:
        {
            // Begin temperature sampling
            transaction.write(REG_CONVERT_D2_4096);
            deviceState = STATE_SAMPLED_TEMP;
            break;
        }
        case STATE_SAMPLED_TEMP:
        {
            // Read back the sampled temperature
            transaction.read(REG_ADC_READ, buffer, 3, false);

            uint32_t tmpRawTemperature = (uint32_t)buffer[2] |
                                         ((uint32_t)buffer[1] << 8) |
                                         ((uint32_t)buffer[0] << 16);
            lastTemperatureTimestamp = TimestampTimer::getTimestamp();

            // Check if the value is valid
            if (tmpRawTemperature != 0)
            {
                rawTemperature = tmpRawTemperature;
            }
            else
            {
                LOG_ERR(logger, "The read raw temperature isn't valid");
            }

            // Begin pressure sampling
            transaction.write(REG_CONVERT_D1_4096);
            deviceState = STATE_SAMPLED_PRESS;
            break;
        }
        case STATE_SAMPLED_PRESS:
        {
            // Read back the sampled pressure
            transaction.read(REG_ADC_READ, buffer, 3, false);

            uint32_t tmpRawPressure = (uint32_t)buffer[2] |
                                      ((uint32_t)buffer[1] << 8) |
                                      ((uint32_t)buffer[0] << 16);

            // Check if the value is valid
            if (tmpRawPressure != 0)
            {
                rawPressure = tmpRawPressure;
            }
            else
            {
                LOG_ERR(logger, "The read raw pressure isn't valid");
            }

            last_sample = updateData();

            // Check whether to read the pressure or the temperature
            tempCounter++;
            if (tempCounter % temperatureDivider == 0)
            {
                // Begin temperature sampling
                transaction.write(REG_CONVERT_D2_4096);
                deviceState = STATE_SAMPLED_TEMP;
            }
            else
            {
                // Begin pressure sampling again
                transaction.write(REG_CONVERT_D1_4096);
            }
            break;
        }
        default:
            break;
    }

    return last_sample;
}

MS5803Data MS5803::updateData()
{
    // First order compensation
    int32_t dt   = rawTemperature - (((uint32_t)calibrationData.tref) << 8);
    int32_t temp = 2000 + (((uint64_t)dt * calibrationData.tempsens) >> 23);

    int64_t offs = ((int64_t)calibrationData.off << 16) +
                   (((int64_t)calibrationData.tco * dt) >> 7);
    int64_t sens = ((int64_t)calibrationData.sens << 15) +
                   (((int64_t)calibrationData.tcs * dt) >> 8);

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

    float pressure =
        (((((int64_t)rawPressure) * sens) / 2097152.0) - offs) / 32786.0;

    // Pressure in Pascal
    float temp_ = temp / 100.0f;

    return MS5803Data(TimestampTimer::getTimestamp(), pressure,
                      lastTemperatureTimestamp, temp_);
}

uint16_t MS5803::readReg(SPITransaction& transaction, uint8_t reg)
{
    uint8_t rcv[2];
    transaction.read(reg, rcv, 2);
    uint16_t data = (rcv[0] << 8) | rcv[1];
    return data;
}

}  // namespace Boardcore
