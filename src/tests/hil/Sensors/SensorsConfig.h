/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>

namespace HILTest
{
namespace SensorsConfig
{
constexpr Boardcore::LPS22DF::AVG LPS22DF_AVG = Boardcore::LPS22DF::AVG_4;
constexpr Boardcore::LPS22DF::ODR LPS22DF_ODR = Boardcore::LPS22DF::ODR_100;

constexpr Boardcore::LPS28DFW::AVG LPS28DFW_AVG = Boardcore::LPS28DFW::AVG_4;
constexpr Boardcore::LPS28DFW::ODR LPS28DFW_ODR = Boardcore::LPS28DFW::ODR_100;
constexpr Boardcore::LPS28DFW::FullScaleRange LPS28DFW_FSR =
    Boardcore::LPS28DFW::FS_1260;

constexpr Boardcore::H3LIS331DLDefs::OutputDataRate H3LIS331DL_ODR =
    Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
constexpr Boardcore::H3LIS331DLDefs::BlockDataUpdate H3LIS331DL_BDU =
    Boardcore::H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE;
constexpr Boardcore::H3LIS331DLDefs::FullScaleRange H3LIS331DL_FSR =
    Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;

constexpr Boardcore::LIS2MDL::OperativeMode LIS2MDL_OPERATIVE_MODE =
    Boardcore::LIS2MDL::MD_CONTINUOUS;
constexpr Boardcore::LIS2MDL::ODR LIS2MDL_ODR = Boardcore::LIS2MDL::ODR_100_HZ;
constexpr unsigned int LIS2MDL_TEMPERATURE_DIVIDER = 5;

constexpr uint8_t UBXGPS_SAMPLE_RATE = 10;

constexpr Boardcore::LSM6DSRXConfig::BDU LSM6DSRX_BDU =
    Boardcore::LSM6DSRXConfig::BDU::CONTINUOUS_UPDATE;

constexpr Boardcore::LSM6DSRXConfig::ACC_FULLSCALE LSM6DSRX_ACC_FS =
    Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G16;
constexpr Boardcore::LSM6DSRXConfig::ACC_ODR LSM6DSRX_ACC_ODR =
    Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_416;
constexpr Boardcore::LSM6DSRXConfig::OPERATING_MODE LSM6DSRX_OPERATING_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::NORMAL;

constexpr Boardcore::LSM6DSRXConfig::GYR_FULLSCALE LSM6DSRX_GYR_FS =
    Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_4000;
constexpr Boardcore::LSM6DSRXConfig::GYR_ODR LSM6DSRX_GYR_ODR =
    Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_416;

constexpr Boardcore::LSM6DSRXConfig::FIFO_MODE LSM6DSRX_FIFO_MODE =
    Boardcore::LSM6DSRXConfig::FIFO_MODE::CONTINUOUS;
constexpr Boardcore::LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION
    LSM6DSRX_FIFO_TIMESTAMP_DECIMATION =
        Boardcore::LSM6DSRXConfig::FIFO_TIMESTAMP_DECIMATION::DEC_1;
constexpr Boardcore::LSM6DSRXConfig::FIFO_TEMPERATURE_BDR
    LSM6DSRX_FIFO_TEMPERATURE_BDR =
        Boardcore::LSM6DSRXConfig::FIFO_TEMPERATURE_BDR::DISABLED;

constexpr Boardcore::ADS131M08Defs::OversamplingRatio
    ADS131M08_OVERSAMPLING_RATIO =
        Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool ADS131M08_GLOBAL_CHOP_MODE = true;

constexpr uint32_t LPS22DF_PERIOD         = 20;  // [ms] 50Hz
constexpr uint32_t LPS28DFW_PERIOD        = 20;  // [ms] 50Hz
constexpr uint32_t H3LIS331DL_PERIOD      = 10;  // [ms] 100Hz
constexpr uint32_t LIS2MDL_PERIOD         = 10;  // [ms] 100Hz
constexpr uint32_t UBXGPS_PERIOD          = 1000 / UBXGPS_SAMPLE_RATE;  // [ms]
constexpr uint32_t LSM6DSRX_PERIOD        = 10;  // [ms] 100Hz
constexpr uint32_t ADS131M08_PERIOD       = 20;  // [ms] 50Hz
constexpr uint32_t IMU_PERIOD             = 20;  // [ms] Fake processed IMU 50Hz
constexpr uint32_t MAG_CALIBRATION_PERIOD = 20;  // [ms] 50Hz

// ADC sensors configs
constexpr float ADC_VOLTAGE_RANGE = 1.2f;  // [V] Voltage reading range
constexpr Boardcore::ADS131M08Defs::Channel ADC_CH_DPL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_3;
constexpr Boardcore::ADS131M08Defs::Channel ADC_CH_STATIC_1 =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_4;
constexpr Boardcore::ADS131M08Defs::Channel ADC_CH_STATIC_2 =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_2;

// ADC Voltage divider
constexpr float BATTERY_VOLTAGE_CONVERSION_FACTOR =
    (20.f / 2.4f) +
    1;  // 20 kOhm resistor and 2.4 kOhm resistor for voltage divider
constexpr float CURRENT_CONVERSION_FACTOR =
    (20.f / 4.f) / (12.f / (12.f + 33.f));
constexpr float CURRENT_OFFSET = 0.133333333f;  // V in ADC

// Calibration samples
constexpr unsigned int CALIBRATION_SAMPLES = 20;
constexpr unsigned int CALIBRATION_PERIOD  = 100;

constexpr unsigned int NUMBER_OF_SENSORS = 8;

}  // namespace SensorsConfig
}  // namespace HILTest
