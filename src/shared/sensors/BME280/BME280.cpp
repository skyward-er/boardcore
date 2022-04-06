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

#include "BME280.h"

#include <drivers/timer/TimestampTimer.h>
#include <math.h>

using namespace std;

namespace Boardcore
{

const BME280::BME280Config BME280::BME280_DEFAULT_CONFIG = {
    SKIPPED, 0, 0, SLEEP_MODE, SKIPPED, SKIPPED, 0, FILTER_OFF, STB_TIME_0_5};

const BME280::BME280Config BME280::BME280_CONFIG_ALL_ENABLED = {OVERSAMPLING_1,
                                                                0,
                                                                0,
                                                                NORMAL_MODE,
                                                                OVERSAMPLING_16,
                                                                OVERSAMPLING_2,
                                                                0,
                                                                FILTER_COEFF_16,
                                                                STB_TIME_0_5};

const BME280::BME280Config BME280::BME280_CONFIG_TEMP_SINGLE = {
    SKIPPED,        0, 0,          FORCED_MODE, SKIPPED,
    OVERSAMPLING_1, 0, FILTER_OFF, STB_TIME_0_5};

BME280::BME280(SPISlave spiSlave, BME280Config config)
    : spiSlave(spiSlave), config(config)
{
}

bool BME280::init()
{
    // Check WHO AM I
    if (!checkWhoAmI())
    {
        LOG_ERR(logger, "Invalid WHO AM I");

        lastError = SensorErrors::INVALID_WHOAMI;

        return false;
    }

    loadCompensationParameters();

    // Read once the temperature to compute fineTemperature
    setConfiguration(BME280_CONFIG_TEMP_SINGLE);
    miosix::Thread::sleep(
        calculateMaxMeasurementTime(BME280_CONFIG_TEMP_SINGLE));
    readTemperature();

    setConfiguration();

    BME280Config readBackConfig = readConfiguration();

    // Check if the configration on the device matches ours
    if (config.bytes.ctrlHumidity != readBackConfig.bytes.ctrlHumidity ||
        config.bytes.ctrlPressureAndTemperature !=
            readBackConfig.bytes.ctrlPressureAndTemperature ||
        config.bytes.config != readBackConfig.bytes.config)
    {
        LOG_ERR(logger, "Device configuration incorrect, setup failed");

        lastError = SensorErrors::NOT_INIT;

        return false;
    }

    return true;
}

void BME280::setSensorMode(Mode mode)
{
    config.bits.mode = mode;

    setConfiguration();
}

void BME280::setHumidityOversampling(Oversampling oversampling)
{
    config.bits.oversamplingHumidity = oversampling;

    setConfiguration();
}

void BME280::setPressureOversampling(Oversampling oversampling)
{
    config.bits.oversamplingPressure = oversampling;

    setConfiguration();
}

void BME280::setTemperatureOversampling(Oversampling oversampling)
{
    config.bits.oversamplingTemperature = oversampling;

    setConfiguration();
}

void BME280::setFilterCoeff(FilterCoeff filterCoeff)
{
    config.bits.filter = filterCoeff;

    setConfiguration();
}

void BME280::setStandbyTime(StandbyTime standbyTime)
{
    config.bits.standbyTime = standbyTime;

    setConfiguration();
}

HumidityData BME280::readHumidity()
{
    uint8_t buffer[2];
    int32_t adc_H = 0;

    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_HUM_MSB, buffer, 2);
    }

    adc_H |= ((uint32_t)buffer[0] << 8);
    adc_H |= buffer[1];

    // Compensate humidity
    lastSample.humidityTimestamp = TimestampTimer::getInstance().getTimestamp();
    lastSample.humidity =
        (float)compensateHumidity(adc_H) / 1024;  // Converto to %RH

    return lastSample;
}

PressureData BME280::readPressure()
{
    uint8_t buffer[3];
    int32_t adc_P = 0;

    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_PRESS_MSB, buffer, 3);
    }

    adc_P |= ((uint32_t)buffer[0]) << 12;
    adc_P |= ((uint32_t)buffer[1]) << 4;
    adc_P |= (buffer[2] >> 4) & 0x0F;

    // Compensate pressure
    lastSample.pressureTimestamp = TimestampTimer::getInstance().getTimestamp();
    lastSample.pressure =
        (float)compensatePressure(adc_P) / 256;  // Convert to Pa

    return lastSample;
}

TemperatureData BME280::readTemperature()
{
    uint8_t buffer[3];
    int32_t adcTemperature = 0;

    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_TEMP_MSB, buffer, 3);
    }

    adcTemperature |= ((uint32_t)buffer[0]) << 12;
    adcTemperature |= ((uint32_t)buffer[1]) << 4;
    adcTemperature |= (buffer[2] >> 4) & 0x0F;

    // Compensate temperature
    fineTemperature = computeFineTemperature(adcTemperature);
    lastSample.temperatureTimestamp =
        TimestampTimer::getInstance().getTimestamp();
    lastSample.temperature = (float)compensateTemperature(fineTemperature) /
                             100;  // Converto to DegC

    return lastSample;
}

unsigned int BME280::calculateMaxMeasurementTime(BME280Config config_)
{
    return ceil(1.25 + (2.3 * config_.bits.oversamplingTemperature) +
                (2.3 * config_.bits.oversamplingPressure + 0.575) +
                (2.3 * config_.bits.oversamplingHumidity + 0.575));
}

unsigned int BME280::getMaxMeasurementTime()
{
    return calculateMaxMeasurementTime(config);
}

bool BME280::selfTest() { return checkWhoAmI(); }

BME280Data BME280::sampleImpl()
{
    uint8_t buffer[8];
    int32_t adcTemperature = 0;
    int32_t adc_P          = 0;
    int32_t adc_H          = 0;
    BME280Data data;

    // TODO: implement selective read!

    // Burst read pressure, temperature and humidity
    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_PRESS_MSB, buffer, 8);
    }

    adcTemperature |= ((uint32_t)buffer[3]) << 12;
    adcTemperature |= ((uint32_t)buffer[4]) << 4;
    adcTemperature |= (buffer[5] >> 4) & 0x0F;

    adc_P |= ((uint32_t)buffer[0]) << 12;
    adc_P |= ((uint32_t)buffer[1]) << 4;
    adc_P |= (buffer[2] >> 4) & 0x0F;

    adc_H |= ((uint32_t)buffer[6] << 8);
    adc_H |= buffer[7];

    // Compensate temperature
    fineTemperature           = computeFineTemperature(adcTemperature);
    data.temperatureTimestamp = TimestampTimer::getInstance().getTimestamp();
    data.temperature          = (float)compensateTemperature(fineTemperature) /
                       100;  // Converto to DegC

    // Compensate pressure
    data.pressureTimestamp = TimestampTimer::getInstance().getTimestamp();
    data.pressure = (float)compensatePressure(adc_P) / 256;  // Convert to Pa

    // Compensate humidity
    data.humidityTimestamp = TimestampTimer::getInstance().getTimestamp();
    data.humidity = (float)compensateHumidity(adc_H) / 1024;  // Converto to %RH

    return data;
}

bool BME280::checkWhoAmI()
{
    SPITransaction transaction(spiSlave);

    uint8_t whoAmIValue = transaction.readRegister(REG_ID);

    return whoAmIValue == REG_ID_VAL;
}

void BME280::setConfiguration() { setConfiguration(config); }

void BME280::setConfiguration(BME280Config config_)
{
    SPITransaction transaction(spiSlave);

    transaction.writeRegister(REG_CONFIG & 0x7F, config_.bytes.config);
    transaction.writeRegister(REG_CTRL_HUM & 0x7F, config_.bytes.ctrlHumidity);
    transaction.writeRegister(REG_CTRL_MEAS & 0x7F,
                              config_.bytes.ctrlPressureAndTemperature);
}

BME280::BME280Config BME280::readConfiguration()
{
    BME280Config tmp;
    SPITransaction transaction(spiSlave);

    transaction.readRegisters(REG_CTRL_HUM, (uint8_t *)&tmp, 4);

    return tmp;
}

void BME280::loadCompensationParameters()
{
    // Read first batch of compensation parameters
    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_CALIB_0, (uint8_t *)&compParams, 25);
    }

    // Reat second batch of compensation parameters
    {
        SPITransaction transaction(spiSlave);

        transaction.readRegisters(REG_CALIB_26,
                                  (uint8_t *)&compParams.bits.dig_H2, 7);
    }

    // Adjust unaligned data
    compParams.bytesArray[29] =
        (compParams.bytesArray[29] << 4) | (compParams.bytesArray[29] >> 4);
    compParams.bits.dig_H4 =
        (compParams.bits.dig_H4 << 4) | (compParams.bits.dig_H4 >> 8);
}

int32_t BME280::computeFineTemperature(int32_t adcTemperature)
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

int32_t BME280::compensateTemperature(int32_t fineTemperature)
{
    return (fineTemperature * 5 + 128) >> 8;
}

uint32_t BME280::compensatePressure(int32_t adc_P)
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

uint32_t BME280::compensateHumidity(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = (fineTemperature - ((int32_t)768000));
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

}  // namespace Boardcore
