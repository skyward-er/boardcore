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

#include "BME280I2C.h"

#include <drivers/timer/TimestampTimer.h>
#include <math.h>

using namespace std;

namespace Boardcore
{

const BME280I2C::BME280Config BME280I2C::BME280_DEFAULT_CONFIG = {
    SKIPPED, 0, 0, SLEEP_MODE, SKIPPED, SKIPPED, 0, FILTER_OFF, STB_TIME_0_5};

const BME280I2C::BME280Config BME280I2C::BME280_CONFIG_ALL_ENABLED = {
    OVERSAMPLING_1,
    0,
    0,
    NORMAL_MODE,
    OVERSAMPLING_16,
    OVERSAMPLING_2,
    0,
    FILTER_COEFF_16,
    STB_TIME_0_5};

const BME280I2C::BME280Config BME280I2C::BME280_CONFIG_TEMP_SINGLE = {
    SKIPPED,        0, 0,          FORCED_MODE, SKIPPED,
    OVERSAMPLING_1, 0, FILTER_OFF, STB_TIME_0_5};

BME280I2C::BME280I2C(I2C &bus, BME280Config config) : bus(bus), config(config)
{
}

bool BME280I2C::init()
{
    if (!checkWhoAmI())
    {
        LOG_ERR(logger, "Invalid WHO AM I");

        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    if (!reset())
    {
        return false;
    }
    miosix::Thread::sleep(3);

    loadCompensationParameters();

    // Read once the temperature to compute fineTemperature
    setConfiguration(BME280_CONFIG_TEMP_SINGLE);
    miosix::Thread::sleep(
        calculateMaxMeasurementTime(BME280_CONFIG_TEMP_SINGLE));
    readTemperature();

    // Set the target configuration
    setConfiguration();

    BME280Config readBackConfig = readConfiguration();

    // Check if the configuration on the device matches ours
    if (config.bytes.ctrlHumidity != readBackConfig.bytes.ctrlHumidity ||
        config.bytes.ctrlPressureAndTemperature !=
            readBackConfig.bytes.ctrlPressureAndTemperature ||
        config.bytes.config != readBackConfig.bytes.config)
    {
        LOG_ERR(logger, "Device configuration incorrect, setup failed.");

        lastError = SensorErrors::NOT_INIT;
        return false;
    }

    return true;
}

void BME280I2C::setSensorMode(Mode mode)
{
    config.bits.mode = mode;

    setConfiguration();
}

void BME280I2C::setHumidityOversampling(Oversampling oversampling)
{
    config.bits.oversamplingHumidity = oversampling;

    setConfiguration();
}

void BME280I2C::setPressureOversampling(Oversampling oversampling)
{
    config.bits.oversamplingPressure = oversampling;

    setConfiguration();
}

void BME280I2C::setTemperatureOversampling(Oversampling oversampling)
{
    config.bits.oversamplingTemperature = oversampling;

    setConfiguration();
}

void BME280I2C::setFilterCoeff(FilterCoeff filterCoeff)
{
    config.bits.filter = filterCoeff;

    setConfiguration();
}

void BME280I2C::setStandbyTime(StandbyTime standbyTime)
{
    config.bits.standbyTime = standbyTime;

    setConfiguration();
}

HumidityData BME280I2C::readHumidity()
{
    uint8_t buffer[2];
    if (bus.readFromRegister(slaveConfig, REG_HUM_MSB, buffer, 2))
    {

        int32_t adc_H = ((uint32_t)buffer[0] << 8);
        adc_H |= buffer[1];

        HumidityData data;
        data.humidityTimestamp = TimestampTimer::getTimestamp();
        data.humidity          = compensateHumidity(adc_H);
        data.humidity /= 1024;  // Convert to to %RH

        return data;
    }
    else
    {
        lastError = SensorErrors::BUS_FAULT;
        return lastSample;
    }
}

PressureData BME280I2C::readPressure()
{
    uint8_t buffer[3];
    if (bus.readFromRegister(slaveConfig, REG_PRESS_MSB, buffer, 3))
    {

        int32_t adc_P = ((uint32_t)buffer[0]) << 12;
        adc_P |= ((uint32_t)buffer[1]) << 4;
        adc_P |= (buffer[2] >> 4) & 0x0F;

        PressureData data;
        data.pressureTimestamp = TimestampTimer::getTimestamp();
        data.pressure          = compensatePressure(adc_P);
        data.pressure /= 256;  // Convert to Pa

        return data;
    }
    else
    {
        lastError = SensorErrors::BUS_FAULT;
        return lastSample;
    }
}

TemperatureData BME280I2C::readTemperature()
{
    uint8_t buffer[3];
    if (bus.readFromRegister(slaveConfig, REG_TEMP_MSB, buffer, 3))
    {
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
    else
    {
        lastError = SensorErrors::BUS_FAULT;
        return lastSample;
    }
}

unsigned int BME280I2C::calculateMaxMeasurementTime(BME280Config config)
{
    return ceil(1.25 + (2.3 * config.bits.oversamplingTemperature) +
                (2.3 * config.bits.oversamplingPressure + 0.575) +
                (2.3 * config.bits.oversamplingHumidity + 0.575));
}

unsigned int BME280I2C::getMaxMeasurementTime()
{
    return calculateMaxMeasurementTime(config);
}

bool BME280I2C::selfTest() { return checkWhoAmI(); }

BME280Data BME280I2C::sampleImpl()
{
    // TODO: implement selective read!

    uint8_t buffer[8];
    if (bus.readFromRegister(slaveConfig, REG_PRESS_MSB, buffer, 8))
    {
        BME280Data data;

        int32_t adcTemperature = ((uint32_t)buffer[3]) << 12;
        adcTemperature |= ((uint32_t)buffer[4]) << 4;
        adcTemperature |= (buffer[5] >> 4) & 0x0F;

        int32_t adc_P = ((uint32_t)buffer[0]) << 12;
        adc_P |= ((uint32_t)buffer[1]) << 4;
        adc_P |= (buffer[2] >> 4) & 0x0F;

        int32_t adc_H = ((uint32_t)buffer[6] << 8);
        adc_H |= buffer[7];

        // Compensate temperature
        fineTemperature           = computeFineTemperature(adcTemperature);
        data.temperatureTimestamp = TimestampTimer::getTimestamp();
        data.temperature          = compensateTemperature(fineTemperature);
        data.temperature /= 100;  // Convert to to DegC

        // Compensate pressure
        data.pressureTimestamp = TimestampTimer::getTimestamp();
        data.pressure          = compensatePressure(adc_P);
        data.pressure /= 256;  // Convert to Pa

        // Compensate humidity
        data.humidityTimestamp = TimestampTimer::getTimestamp();
        data.humidity          = compensateHumidity(adc_H);
        data.humidity /= 1024;  // Convert to to %RH

        return data;
    }
    else
    {
        lastError = SensorErrors::BUS_FAULT;
        return lastSample;
    }
}

bool BME280I2C::reset()
{
    if (!bus.writeRegister(slaveConfig, REG_RESET, 0xB6))
    {
        lastError = SensorErrors::BUS_FAULT;
        return false;
    }

    return true;
}

bool BME280I2C::checkWhoAmI()
{
    uint8_t whoAmIValue;

    if (bus.readRegister(slaveConfig, REG_ID, whoAmIValue))
    {

        return whoAmIValue == REG_ID_VAL;
    }
    else
    {
        lastError = SensorErrors::BUS_FAULT;
        return false;
    }
}

void BME280I2C::setConfiguration() { setConfiguration(config); }

void BME280I2C::setConfiguration(BME280Config config)
{
    if (!bus.writeRegister(slaveConfig, REG_CONFIG, config.bytes.config))
    {
        LOG_ERR(logger, "Error while writing to register REG_CONFIG");
        return;
    }

    if (!bus.writeRegister(slaveConfig, REG_CTRL_HUM,
                           config.bytes.ctrlHumidity))
    {
        LOG_ERR(logger, "Error while writing to register REG_CTRL_HUM");
        return;
    }

    if (!bus.writeRegister(slaveConfig, REG_CTRL_MEAS,
                           config.bytes.ctrlPressureAndTemperature))
    {
        LOG_ERR(logger, "Error while writing to register REG_CTRL_MEAS");
        return;
    }
}

BME280I2C::BME280Config BME280I2C::readConfiguration()
{
    BME280Config tmp;

    if (bus.readFromRegister(slaveConfig, REG_CTRL_HUM,
                             reinterpret_cast<uint8_t *>(&tmp), 4))
    {
        return tmp;
    }
    else
    {
        lastError = SensorErrors::BUS_FAULT;
        return BME280_DEFAULT_CONFIG;
    }
}

void BME280I2C::loadCompensationParameters()
{
    // Read first batch of compensation parameters
    if (!bus.readFromRegister(slaveConfig, REG_CALIB_0,
                              reinterpret_cast<uint8_t *>(&compParams), 25))
    {
        lastError = SensorErrors::BUS_FAULT;
        return;
    }

    // Read second batch of compensation parameters
    if (!bus.readFromRegister(
            slaveConfig, REG_CALIB_26,
            reinterpret_cast<uint8_t *>(&compParams.bits.dig_H2), 7))
    {
        lastError = SensorErrors::BUS_FAULT;
        return;
    }

    // Adjust unaligned data
    compParams.bytesArray[29] =
        (compParams.bytesArray[29] << 4) | (compParams.bytesArray[29] >> 4);
    compParams.bits.dig_H4 =
        (compParams.bits.dig_H4 << 4) | (compParams.bits.dig_H4 >> 8);
}

int32_t BME280I2C::computeFineTemperature(int32_t adcTemperature)
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

int32_t BME280I2C::compensateTemperature(int32_t fineTemperature)
{
    return (fineTemperature * 5 + 128) >> 8;
}

uint32_t BME280I2C::compensatePressure(int32_t adc_P)
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

uint32_t BME280I2C::compensateHumidity(int32_t adc_H)
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
