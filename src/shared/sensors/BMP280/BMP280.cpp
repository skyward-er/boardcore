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

#include "BMP280.h"

#include <drivers/timer/TimestampTimer.h>
#include <math.h>

using namespace std;

namespace Boardcore
{

const BMP280::BMP280Config BMP280::BMP280_DEFAULT_CONFIG = {
    0, 0, SLEEP_MODE, SKIPPED, SKIPPED, 0, FILTER_OFF, STB_TIME_0_5};

const BMP280::BMP280Config BMP280::BMP280_CONFIG_ALL_ENABLED = {0,
                                                                0,
                                                                NORMAL_MODE,
                                                                OVERSAMPLING_16,
                                                                OVERSAMPLING_2,
                                                                0,
                                                                FILTER_COEFF_16,
                                                                STB_TIME_0_5};

const BMP280::BMP280Config BMP280::BMP280_CONFIG_TEMP_SINGLE = {
    0, 0, FORCED_MODE, SKIPPED, OVERSAMPLING_1, 0, FILTER_OFF, STB_TIME_0_5};

BMP280::BMP280(SPISlave spiSlave, BMP280Config config)
    : spiSlave(spiSlave), config(config)
{
}

bool BMP280::init()
{
    if (!checkWhoAmI())
    {
        LOG_ERR(logger, "Invalid WHO AM I");

        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    reset();
    miosix::Thread::sleep(3);

    loadCompensationParameters();

    // Read once the temperature to compute fineTemperature
    setConfiguration(BMP280_CONFIG_TEMP_SINGLE);
    miosix::Thread::sleep(
        calculateMaxMeasurementTime(BMP280_CONFIG_TEMP_SINGLE));
    readTemperature();

    // Set the target configuration
    setConfiguration();

    BMP280Config readBackConfig = readConfiguration();

    // Check if the configuration on the device matches ours
    if (config.bytes.ctrlPressureAndTemperature !=
            readBackConfig.bytes.ctrlPressureAndTemperature ||
        config.bytes.config != readBackConfig.bytes.config)
    {
        LOG_ERR(logger, "Device configuration incorrect, setup failed");

        lastError = SensorErrors::NOT_INIT;
        return false;
    }

    return true;
}

void BMP280::setSensorMode(Mode mode)
{
    config.bits.mode = mode;

    setConfiguration();
}

void BMP280::setPressureOversampling(Oversampling oversampling)
{
    config.bits.oversamplingPressure = oversampling;

    setConfiguration();
}

void BMP280::setTemperatureOversampling(Oversampling oversampling)
{
    config.bits.oversamplingTemperature = oversampling;

    setConfiguration();
}

void BMP280::setFilterCoeff(FilterCoeff filterCoeff)
{
    config.bits.filter = filterCoeff;

    setConfiguration();
}

void BMP280::setStandbyTime(StandbyTime standbyTime)
{
    config.bits.standbyTime = standbyTime;

    setConfiguration();
}

PressureData BMP280::readPressure()
{
    using namespace Units::Pressure;

    uint8_t buffer[3];

    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_PRESS_MSB, buffer, 3);
    }

    int32_t adc_P = ((uint32_t)buffer[0]) << 12;
    adc_P |= ((uint32_t)buffer[1]) << 4;
    adc_P |= (buffer[2] >> 4) & 0x0F;

    PressureData data;
    data.pressureTimestamp = TimestampTimer::getTimestamp();
    data.pressure          = Pascal(compensatePressure(adc_P));
    data.pressure /= 256;  // Convert to Pa

    return data;
}

TemperatureData BMP280::readTemperature()
{
    uint8_t buffer[3];

    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_TEMP_MSB, buffer, 3);
    }

    int32_t adcTemperature = ((uint32_t)buffer[0]) << 12;
    adcTemperature |= ((uint32_t)buffer[1]) << 4;
    adcTemperature |= (buffer[2] >> 4) & 0x0F;

    fineTemperature = computeFineTemperature(adcTemperature);

    TemperatureData data;
    data.temperatureTimestamp = TimestampTimer::getTimestamp();
    data.temperature          = compensateTemperature(fineTemperature);
    data.temperature /= 100;  // Convert to to DegC

    return data;
}

unsigned int BMP280::calculateMaxMeasurementTime(BMP280Config config)
{
    // TODO: This formula is not present in the BMP280's datasheet, it should
    // be checked
    return ceil(1.25 + (2.3 * config.bits.oversamplingTemperature) +
                (2.3 * config.bits.oversamplingPressure + 0.575));
}

unsigned int BMP280::getMaxMeasurementTime()
{
    return calculateMaxMeasurementTime(config);
}

bool BMP280::selfTest() { return checkWhoAmI(); }

BMP280Data BMP280::sampleImpl()
{
    using namespace Units::Pressure;

    // TODO: implement selective read!

    uint8_t buffer[8];
    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_PRESS_MSB, buffer, 8);
    }

    BMP280Data data;

    int32_t adcTemperature = ((uint32_t)buffer[3]) << 12;
    adcTemperature |= ((uint32_t)buffer[4]) << 4;
    adcTemperature |= (buffer[5] >> 4) & 0x0F;

    int32_t adc_P = ((uint32_t)buffer[0]) << 12;
    adc_P |= ((uint32_t)buffer[1]) << 4;
    adc_P |= (buffer[2] >> 4) & 0x0F;

    // Compensate temperature
    fineTemperature           = computeFineTemperature(adcTemperature);
    data.temperatureTimestamp = TimestampTimer::getTimestamp();
    data.temperature          = compensateTemperature(fineTemperature);
    data.temperature /= 100;  // Convert to to DegC

    // Compensate pressure
    data.pressureTimestamp = TimestampTimer::getTimestamp();
    data.pressure          = Pascal(compensatePressure(adc_P));
    data.pressure /= 256;  // Convert to Pa

    return data;
}

void BMP280::reset()
{
    SPITransaction transaction(spiSlave);

    transaction.writeRegister(REG_RESET, 0xB6);
}

bool BMP280::checkWhoAmI()
{
    SPITransaction transaction(spiSlave);

    uint8_t whoAmIValue = transaction.readRegister(REG_ID);

    return whoAmIValue == REG_ID_VAL;
}

void BMP280::setConfiguration() { setConfiguration(config); }

void BMP280::setConfiguration(BMP280Config config)
{
    SPITransaction transaction(spiSlave);

    transaction.writeRegister(REG_CONFIG, config.bytes.config);
    transaction.writeRegister(REG_CTRL_MEAS,
                              config.bytes.ctrlPressureAndTemperature);
}

BMP280::BMP280Config BMP280::readConfiguration()
{
    BMP280Config tmp;
    SPITransaction transaction(spiSlave);

    transaction.readRegisters(REG_STATUS, (uint8_t *)&tmp, 3);

    return tmp;
}

void BMP280::loadCompensationParameters()
{
    // Read compensation parameters
    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_CALIB_0, (uint8_t *)&compParams, 25);
    }
}

int32_t BMP280::computeFineTemperature(int32_t adcTemperature)
{
    int32_t var1, var2;
    var1 = ((((adcTemperature >> 3) - ((int32_t)compParams.bits.dig_T1 << 1))) *
            ((int32_t)compParams.bits.dig_T2)) >>
           11;
    var2 = (((((adcTemperature >> 4) - ((int32_t)compParams.bits.dig_T1)) *
              ((adcTemperature >> 4) - ((int32_t)compParams.bits.dig_T1))) >>
             12) *
            ((int32_t)compParams.bits.dig_T3)) >>
           14;
    return var1 + var2;
}

int32_t BMP280::compensateTemperature(int32_t fineTemperature)
{
    return (fineTemperature * 5 + 128) >> 8;
}

uint32_t BMP280::compensatePressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)fineTemperature) - 128000;
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

}  // namespace Boardcore
