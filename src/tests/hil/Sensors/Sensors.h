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
#include <sensors/SensorData.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <sensors/analog/BatteryVoltageSensorData.h>
#include <sensors/analog/Pitot/PitotData.h>
#include <sensors/analog/pressure/honeywell/HSCMRNN015PA.h>
#include <sensors/analog/pressure/nxp/MPXH6400A.h>
#include <sensors/calibration/SoftAndHardIronCalibration/SoftAndHardIronCalibration.h>
#include <stdint.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "../Buses.h"
#include "RotatedIMU/RotatedIMU.h"
#include "SensorsConfig.h"
#include "SensorsData.h"

namespace HILTest
{
class Sensors : public Boardcore::Module
{
public:
    explicit Sensors(Boardcore::TaskScheduler* sched, Buses* buses);

    [[nodiscard]] virtual bool start();

    /**
     * @brief Stops the sensor manager
     * @warning Stops the passed scheduler
     */
    virtual void stop();

    /**
     * @brief Returns if all the sensors are started successfully
     */
    virtual bool isStarted();

    /**
     * @brief Calibrates the sensors with an offset
     */
    virtual void calibrate();

    /**
     * @brief Takes the result of the live magnetometer calibration and applies
     * it to the current calibration + writes it in the csv file
     */
    virtual bool writeMagCalibration();

    // Sensor getters
    virtual Boardcore::LPS22DFData getLPS22DFLastSample();
    virtual Boardcore::LPS28DFWData getLPS28DFW_1LastSample();
    virtual Boardcore::LPS28DFWData getLPS28DFW_2LastSample();
    virtual Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    virtual Boardcore::LIS2MDLData getLIS2MDLLastSample();
    virtual Boardcore::UBXGPSData getGPSLastSample();
    virtual Boardcore::LSM6DSRXData getLSM6DSRXLastSample();
    virtual Boardcore::ADS131M08Data getADS131M08LastSample();

    // Processed getters
    virtual Boardcore::BatteryVoltageSensorData getBatteryVoltageLastSample();
    virtual Boardcore::BatteryVoltageSensorData
    getCamBatteryVoltageLastSample();
    virtual Boardcore::CurrentData getCurrentLastSample();
    virtual Boardcore::MPXH6400AData getDeploymentPressureLastSample();
    virtual Boardcore::HSCMRNN015PAData getStaticPressure1LastSample();
    virtual Boardcore::HSCMRNN015PAData getStaticPressure2LastSample();
    virtual RotatedIMUData getIMULastSample();
    virtual Boardcore::MagnetometerData getCalibratedMagnetometerLastSample();

    // CAN fake sensors setters
    virtual void setPitot(Boardcore::PitotData data);
    virtual void setCCPressure(Boardcore::PressureData data);
    virtual void setBottomTankPressure(Boardcore::PressureData data);
    virtual void setTopTankPressure(Boardcore::PressureData data);
    virtual void setTankTemperature(Boardcore::TemperatureData data);
    virtual void setMotorBatteryVoltage(
        Boardcore::BatteryVoltageSensorData data);
    virtual void setMotorCurrent(Boardcore::CurrentData data);

    // CAN fake sensors getters
    virtual Boardcore::PitotData getPitotLastSample();
    virtual Boardcore::PressureData getCCPressureLastSample();
    virtual Boardcore::PressureData getBottomTankPressureLastSample();
    virtual Boardcore::PressureData getTopTankPressureLastSample();
    virtual Boardcore::TemperatureData getTankTemperatureLastSample();
    virtual Boardcore::BatteryVoltageSensorData getMotorBatteryVoltage();
    virtual Boardcore::CurrentData getMotorCurrent();

    // Returns the sensors statuses
    std::array<Boardcore::SensorInfo, SensorsConfig::NUMBER_OF_SENSORS>
    getSensorInfo();

protected:
    /**
     * @brief Method to put a sensor in the sensorMap with the relative infos
     */
    template <typename T>
    void registerSensor(Boardcore::Sensor<T>* sensor, const std::string& name,
                        uint32_t period, std::function<void(void)> callback)
    {
        // Emplace the sensor inside the map
        Boardcore::SensorInfo info(name, period, callback);
        sensorMap.emplace(std::make_pair(sensor, info));
    }

    /**
     * @brief Insert a sensor in the infoGetter.
     */
    template <typename T>
    void addInfoGetter(Boardcore::Sensor<T>* sensor)
    {
        // Add the sensor info getter to the array
        sensorsInit[sensorsId++] = [&]() -> Boardcore::SensorInfo
        { return manager->getSensorInfo(sensor); };
    }

    // Creation and callbacks methods
    void lps22dfCreation();
    virtual void lps22dfCallback();

    void lps28dfw_1Creation();
    virtual void lps28dfw_1Callback();

    void lps28dfw_2Creation();
    virtual void lps28dfw_2Callback();

    void h3lis331dlCreation();
    virtual void h3lis331dlCallback();

    void lis2mdlCreation();
    virtual void lis2mdlCallback();

    void ubxgpsCreation();
    virtual void ubxgpsCallback();

    void lsm6dsrxCreation();
    virtual void lsm6dsrxCallback();

    void ads131m08Creation();
    virtual void ads131m08Callback();

    void deploymentPressureCreation();
    virtual void deploymentPressureCallback();

    void staticPressure1Creation();
    virtual void staticPressure1Callback();

    void staticPressure2Creation();
    virtual void staticPressure2Callback();

    void imuCreation();
    virtual void imuCallback();

    // Can sensors
    Boardcore::PitotData canPitot{0, 0, 0};
    Boardcore::PressureData canCCPressure{0, 0};
    Boardcore::PressureData canBottomTankPressure{0, 0};
    Boardcore::PressureData canTopTankPressure{0, 0};
    Boardcore::TemperatureData canTankTemperature{0, 0};
    Boardcore::BatteryVoltageSensorData canMotorBatteryVoltage{};
    Boardcore::CurrentData canMotorCurrent{};

    // Magnetometer live calibration
    Boardcore::SoftAndHardIronCalibration magCalibrator;
    Boardcore::SixParametersCorrector magCalibration;
    miosix::FastMutex calibrationMutex;

    // Fake processed sensors
    std::unique_ptr<RotatedIMU> imu;

    // Sensor manager
    std::unique_ptr<Boardcore::SensorManager> manager;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;
    Buses* buses                        = nullptr;

    // Collection of lambdas to get the sensor init statuses
    std::array<std::function<Boardcore::SensorInfo()>,
               SensorsConfig::NUMBER_OF_SENSORS>
        sensorsInit;
    uint8_t sensorsId = 0;

    // SD logger
    Boardcore::Logger& SDlogger = Boardcore::Logger::getInstance();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");

    // Sensors instances
    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::LPS28DFW> lps28dfw_1;
    std::unique_ptr<Boardcore::LPS28DFW> lps28dfw_2;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl;
    std::unique_ptr<Boardcore::UBXGPSSpi> ubxgps;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx;
    std::unique_ptr<Boardcore::ADS131M08> ads131m08;

    // Fake processed sensors
    std::unique_ptr<Boardcore::MPXH6400A> mpxh6400a;
    std::unique_ptr<Boardcore::HSCMRNN015PA> hscmrnn015pa_1;
    std::unique_ptr<Boardcore::HSCMRNN015PA> hscmrnn015pa_2;
};
}  // namespace HILTest