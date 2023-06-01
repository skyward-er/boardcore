
/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

MS5803::MS5803(SPIBusInterface& spiBus, miosix::GpioPin cs,
               SPIBusConfig spiConfig, uint16_t temperatureDivider)
    : spiSlave(spiBus, cs, spiConfig), temperatureDivider(temperatureDivider)
{
    // Ensure that the write bit is disabled
    spiSlave.config.writeBit = SPI::WriteBit::DISABLED;
}

bool MS5803::init()
{
    SPITransaction transaction{spiSlave};

    // Read calibration data
    calibrationData.sens = transaction.readRegister16(REG_PROM_SENS_MASK);
    calibrationData.off  = transaction.readRegister16(REG_PROM_OFF_MASK);
    calibrationData.tcs  = transaction.readRegister16(REG_PROM_TCS_MASK);
    calibrationData.tco  = transaction.readRegister16(REG_PROM_TCO_MASK);
    calibrationData.tref = transaction.readRegister16(REG_PROM_TREF_MASK);
    calibrationData.tempsens =
        transaction.readRegister16(REG_PROM_TEMPSENS_MASK);

    LOG_INFO(
        logger,
        "sens={:X}, off={:X}, tcs={:X}, tco={:X}, tref={:X}, tempsens={:X}",
        calibrationData.sens, calibrationData.off, calibrationData.tcs,
        calibrationData.tco, calibrationData.tref, calibrationData.tempsens);

    return true;
}

bool MS5803::selfTest() { return true; }

MS5803Data MS5803::sampleImpl()
{
    SPITransaction transaction{spiSlave};

    switch (deviceState)
    {
        case STATE_INIT:
        {
            // Begin temperature sampling
            transaction.write(static_cast<uint8_t>(REG_CONVERT_D2_4096));
            deviceState = STATE_SAMPLED_TEMP;
            break;
        }
        case STATE_SAMPLED_TEMP:
        {
            // Read back the sampled temperature
            uint32_t tmpRawTemperature =
                transaction.readRegister24(REG_ADC_READ);
            lastTemperatureTimestamp = TimestampTimer::getTimestamp();

            // Check if the value is valid
            if (tmpRawTemperature != 0)
                rawTemperature = tmpRawTemperature;
            else
                LOG_ERR(logger, "The read raw temperature isn't valid");

            // Begin pressure sampling
            transaction.write(static_cast<uint8_t>(REG_CONVERT_D1_4096));
            deviceState = STATE_SAMPLED_PRESS;
            break;
        }
        case STATE_SAMPLED_PRESS:
        {
            // Read back the sampled pressure
            uint32_t tmpRawPressure = transaction.readRegister24(REG_ADC_READ);

            // Check if the value is valid
            if (tmpRawPressure != 0)
                rawPressure = tmpRawPressure;
            else
                LOG_ERR(logger, "The read raw pressure isn't valid");

            lastSample = updateData();
            // Check whether to read the pressure or the temperature
            tempCounter++;
            if (tempCounter % temperatureDivider == 0)
            {
                // Begin temperature sampling
                transaction.write(static_cast<uint8_t>(REG_CONVERT_D2_4096));
                deviceState = STATE_SAMPLED_TEMP;
            }
            else
            {
                // Begin pressure sampling again
                transaction.write(static_cast<uint8_t>(REG_CONVERT_D1_4096));
            }
            break;
        }
        default:
            break;
    }

    return lastSample;
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
            sens2 = sens2 + 2 * (temp + 1500) * (temp + 1500);
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
    return MS5803Data(TimestampTimer::getTimestamp(), pressure,
                      lastTemperatureTimestamp, temp / 100.0f);
}

}  // namespace Boardcore
