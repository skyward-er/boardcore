/* Copyright (c) 2015-2022 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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

#include <actuators/Servo/ServoData.h>
#include <actuators/stepper/StepperData.h>
#include <algorithms/ADA/ADAData.h>
#include <algorithms/NAS/NASState.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLoggerData.h>
#include <diagnostic/StackData.h>
#include <drivers/adc/InternalADCData.h>
#include <events/EventData.h>
#include <logger/Deserializer.h>
#include <logger/LoggerStats.h>
#include <radio/MavlinkDriver/MavlinkStatus.h>
#include <radio/Xbee/XbeeStatus.h>
#include <scheduler/TaskSchedulerData.h>
#include <sensors/ADS1118/ADS1118Data.h>
#include <sensors/ADS131M04/ADS131M04Data.h>
#include <sensors/BME280/BME280Data.h>
#include <sensors/BMP280/BMP280Data.h>
#include <sensors/BMX160/BMX160Data.h>
#include <sensors/BMX160/BMX160WithCorrectionData.h>
#include <sensors/HX711/HX711Data.h>
#include <sensors/L3GD20/L3GD20Data.h>
#include <sensors/LIS3DSH/LIS3DSHData.h>
#include <sensors/LIS3MDL/LIS3MDLData.h>
#include <sensors/MBLoadCell/MBLoadCellData.h>
#include <sensors/MPU9250/MPU9250Data.h>
#include <sensors/MS5803/MS5803Data.h>
#include <sensors/SensorData.h>
#include <sensors/UBXGPS/UBXGPSData.h>
#include <sensors/VN100/VN100Data.h>
#include <sensors/analog/AnalogLoadCellData.h>
#include <sensors/analog/BatteryVoltageSensorData.h>
#include <sensors/analog/CurrentSensorData.h>
#include <sensors/analog/pressure/honeywell/HSCMAND015PAData.h>
#include <sensors/analog/pressure/honeywell/HSCMRNN030PAData.h>
#include <sensors/analog/pressure/honeywell/HSCMRNN160KAData.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAAData.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDAData.h>
#include <sensors/analog/pressure/nxp/MPXH6115AData.h>
#include <sensors/analog/pressure/nxp/MPXH6400AData.h>
#include <sensors/analog/pressure/nxp/MPXHZ6130AData.h>

#include <fstream>
#include <iostream>

/**
 * @brief This file includes all the types the logdecoder script will decode.
 *
 * All logged classes inside Boardcore should be reported here.
 */

namespace Boardcore
{

namespace LogTypes
{

void registerTypes(Deserializer& ds)
{
    ds.registerType<StepperData>();
    ds.registerType<ServoData>();
    ds.registerType<ADAState>();
    ds.registerType<NASState>();
    ds.registerType<CpuMeterData>();
    ds.registerType<StackData>();
    ds.registerType<LoggingString>();
    ds.registerType<InternalADCData>();
    ds.registerType<EventData>();
    ds.registerType<LoggerStats>();
    ds.registerType<MavlinkStatus>();
    ds.registerType<Xbee::XbeeStatus>();
    ds.registerType<TaskStatsResult>();
    ds.registerType<ADS1118Data>();
    ds.registerType<ADS131M04Data>();
    ds.registerType<BME280Data>();
    ds.registerType<BMP280Data>();
    ds.registerType<BMX160Data>();
    ds.registerType<BMX160Temperature>();
    ds.registerType<BMX160FifoStats>();
    ds.registerType<BMX160WithCorrectionData>();
    ds.registerType<HX711Data>();
    ds.registerType<L3GD20Data>();
    ds.registerType<LIS3DSHData>();
    ds.registerType<LIS3MDLData>();
    ds.registerType<MBLoadCellData>();
    ds.registerType<MPU9250Data>();
    ds.registerType<MS5803Data>();
    ds.registerType<TemperatureData>();
    ds.registerType<UBXGPSData>();
    ds.registerType<VN100Data>();
    ds.registerType<AnalogLoadCellData>();
    ds.registerType<BatteryVoltageSensorData>();
    ds.registerType<CurrentSensorData>();
    ds.registerType<HSCMAND015PAData>();
    ds.registerType<HSCMRNN030PAData>();
    ds.registerType<HSCMRNN160KAData>();
    ds.registerType<SSCDANN030PAAData>();
    ds.registerType<SSCDRRN015PDAData>();
    ds.registerType<MPXH6115AData>();
    ds.registerType<MPXH6400AData>();
    ds.registerType<MPXHZ6130AData>();
}

}  // namespace LogTypes

}  // namespace Boardcore
