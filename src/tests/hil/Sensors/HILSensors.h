/* Copyright (c) 2023-2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <hil/HIL.h>
#include <sensors/Sensor.h>
#include <sensors/analog/Pitot/Pitot.h>

#include "../HILSimulationConfig.h"
#include "Sensors.h"

namespace HILTest
{

/**
 * @brief Mock sensor which is used as fake sensor for the combustion chamber
 * pressure coming from the canbus.
 */
class MockChamberSensor : public Boardcore::Sensor<Boardcore::PressureData>
{
public:
    explicit MockChamberSensor() {}

    Boardcore::PressureData sampleImpl() override
    {
        return Boardcore::PressureData{0, 0};
    }

protected:
    bool init() override { return true; }

    bool selfTest() override { return true; }
};

class HILSensors : public Sensors
{
public:
    explicit HILSensors(Boardcore::TaskScheduler* sched, Buses* buses,
                        bool enableHw)
        : Sensors{sched, buses}, enableHw{enableHw}
    {
        using namespace HILConfig;
        using namespace Boardcore;

        // Creating the fake sensors for the can transmitted samples
        chamberPressureCreation();
        pitotCreation();

        lps28dfw_1 = hillificator<>(lps28dfw_1, enableHw, updateLPS28DFWData);
        lps28dfw_2 = hillificator<>(lps28dfw_2, enableHw, updateLPS28DFWData);
        lps22df    = hillificator<>(lps22df, enableHw, updateLPS22DFData);
        h3lis331dl = hillificator<>(h3lis331dl, enableHw, updateH3LIS331DLData);
        lis2mdl    = hillificator<>(lis2mdl, enableHw, updateLIS2MDLData);
        ubxgps     = hillificator<>(ubxgps, enableHw, updateUBXGPSData);
        lsm6dsrx   = hillificator<>(lsm6dsrx, enableHw, updateLSM6DSRXData);
        hscmrnn015pa_1 =
            hillificator<>(hscmrnn015pa_1, enableHw, updateStaticPressureData);
        hscmrnn015pa_2 =
            hillificator<>(hscmrnn015pa_2, enableHw, updateStaticPressureData);
        imu     = hillificator<>(imu, enableHw, updateIMUData);
        chamber = hillificator<>(chamber, enableHw, updateCCData);
    };

    bool start() override
    {
        // Registering the fake can sensors
        if (chamber)
        {
            registerSensor(chamber, "chamber", HILConfig::BARO_CHAMBER_PERIOD,
                           [this]() { this->chamberPressureCallback(); });
        }

        if (pitot)
        {
            registerSensor(pitot, "Pitot", HILConfig::PITOT_PERIOD,
                           [this]() { this->pitotCallback(); });
        }

        return Sensors::start();
    }

private:
    void chamberPressureCreation() { chamber = new MockChamberSensor(); }

    void chamberPressureCallback()
    {
        // Warning! Here we must use sensor->getLastSample() instead of
        // getSensorLastSample() since we have to update the "can received
        // value"
        auto lastSample = chamber->getLastSample();
        Boardcore::Logger::getInstance().log(lastSample);
    }

    void pitotCreation()
    {
        pitot =
            new Boardcore::Pitot(getTotalPressurePitot, getStaticPressurePitot);
        pitot->setReferenceValues(Boardcore::ReferenceValues());
    }

    void pitotCallback()
    {
        // Warning! Here we must use sensor->getLastSample() instead of
        // getSensorLastSample() since we have to update the "can received
        // value"
        auto lastSample = pitot->getLastSample();
        Boardcore::Logger::getInstance().log(lastSample);
        setPitot(lastSample);
    }

    static int getSampleCounter(int nData)
    {
        auto ts           = miosix::getTime();
        auto tsSensorData = Boardcore::ModuleManager::getInstance()
                                .get<Boardcore::HILTransceiverBase>()
                                ->getTimestampSimulatorData();
        auto simulationPeriod = Boardcore::ModuleManager::getInstance()
                                    .get<HILConfig::MainHIL>()
                                    ->getSimulationPeriod();

        assert(ts >= tsSensorData &&
               "Actual timestamp is lesser then the packet timestamp");

        // Getting the index floored
        int sampleCounter = (ts - tsSensorData) * nData / simulationPeriod;

        if (sampleCounter >= nData)
        {
            // TODO: Register this as an error
            return nData - 1;  // Return the last valid index
        }

        if (sampleCounter < 0)
        {
            assert(false && "Calculated a negative index");
            return 0;
        }

        return sampleCounter;
    }

    std::function<Boardcore::LPS28DFWData(void)> updateLPS28DFWData = []()
    {
        Boardcore::LPS28DFWData data;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());

        auto* sensorData = hilTransceiver->getSensorData();

        int iBaro = getSampleCounter(sensorData->barometer1.NDATA);
        int iTemp = getSampleCounter(sensorData->temperature.NDATA);

        data.pressureTimestamp = data.temperatureTimestamp = miosix::getTime();
        data.pressure    = sensorData->barometer1.measures[iBaro];
        data.temperature = sensorData->temperature.measures[iTemp];

        return data;
    };

    std::function<Boardcore::LPS22DFData(void)> updateLPS22DFData = []()
    {
        Boardcore::LPS22DFData data;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iBaro = getSampleCounter(sensorData->barometer1.NDATA);
        int iTemp = getSampleCounter(sensorData->temperature.NDATA);

        data.pressureTimestamp = data.temperatureTimestamp = miosix::getTime();
        data.pressure    = sensorData->barometer1.measures[iBaro];
        data.temperature = sensorData->temperature.measures[iTemp];

        return data;
    };

    std::function<Boardcore::H3LIS331DLData(void)> updateH3LIS331DLData = []()
    {
        Boardcore::H3LIS331DLData data;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iAcc = getSampleCounter(sensorData->accelerometer.NDATA);

        data.accelerationTimestamp = miosix::getTime();
        data.accelerationX = sensorData->accelerometer.measures[iAcc][0];
        data.accelerationY = sensorData->accelerometer.measures[iAcc][1];
        data.accelerationZ = sensorData->accelerometer.measures[iAcc][2];

        return data;
    };

    std::function<Boardcore::LIS2MDLData(void)> updateLIS2MDLData = []()
    {
        Boardcore::LIS2MDLData data;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iMag = getSampleCounter(sensorData->magnetometer.NDATA);

        data.magneticFieldTimestamp = miosix::getTime();
        data.magneticFieldX = sensorData->magnetometer.measures[iMag][0];
        data.magneticFieldY = sensorData->magnetometer.measures[iMag][1];
        data.magneticFieldZ = sensorData->magnetometer.measures[iMag][2];

        return data;
    };

    std::function<Boardcore::UBXGPSData(void)> updateUBXGPSData = []()
    {
        Boardcore::UBXGPSData data;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iGps = getSampleCounter(sensorData->gps.NDATA);

        data.gpsTimestamp = miosix::getTime();

        data.latitude  = sensorData->gps.positionMeasures[iGps][0];
        data.longitude = sensorData->gps.positionMeasures[iGps][1];
        data.height    = sensorData->gps.positionMeasures[iGps][2];

        data.velocityNorth = sensorData->gps.velocityMeasures[iGps][0];
        data.velocityEast  = sensorData->gps.velocityMeasures[iGps][1];
        data.velocityDown  = sensorData->gps.velocityMeasures[iGps][2];
        data.speed         = sqrtf(data.velocityNorth * data.velocityNorth +
                                   data.velocityEast * data.velocityEast +
                                   data.velocityDown * data.velocityDown);
        data.positionDOP   = 0;

        data.fix        = static_cast<uint8_t>(sensorData->gps.fix);
        data.satellites = static_cast<uint8_t>(sensorData->gps.num_satellites);

        return data;
    };

    std::function<Boardcore::LSM6DSRXData(void)> updateLSM6DSRXData = []()
    {
        Boardcore::LSM6DSRXData data;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iAcc  = getSampleCounter(sensorData->accelerometer.NDATA);
        int iGyro = getSampleCounter(sensorData->gyro.NDATA);

        data.accelerationTimestamp = data.angularSpeedTimestamp =
            miosix::getTime();

        data.accelerationX = sensorData->accelerometer.measures[iAcc][0];
        data.accelerationY = sensorData->accelerometer.measures[iAcc][1];
        data.accelerationZ = sensorData->accelerometer.measures[iAcc][2];

        data.angularSpeedX = sensorData->gyro.measures[iGyro][0];
        data.angularSpeedY = sensorData->gyro.measures[iGyro][1];
        data.angularSpeedZ = sensorData->gyro.measures[iGyro][2];

        return data;
    };

    std::function<Boardcore::HSCMRNN015PAData(void)> updateStaticPressureData =
        []()
    {
        Boardcore::HSCMRNN015PAData data;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iBaro = getSampleCounter(sensorData->barometer1.NDATA);

        data.pressureTimestamp = miosix::getTime();
        data.pressure          = sensorData->barometer1.measures[iBaro];

        return data;
    };

    std::function<RotatedIMUData(void)> updateIMUData = [this]()
    {
        return RotatedIMUData{getLSM6DSRXLastSample(), getLSM6DSRXLastSample(),
                              getCalibratedMagnetometerLastSample()};
    };

    std::function<Boardcore::PressureData(void)> updateCCData = []()
    {
        Boardcore::PressureData data;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iCC = getSampleCounter(sensorData->pressureChamber.NDATA);

        data.pressureTimestamp = miosix::getTime();
        data.pressure          = sensorData->pressureChamber.measures[iCC];

        return data;
    };

    std::function<float(void)> getTotalPressurePitot = []()
    {
        float totalPressure;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iPitot = getSampleCounter(sensorData->pitot.NDATA);

        totalPressure = sensorData->pitot.staticPressure[iPitot] +
                        sensorData->pitot.deltaP[iPitot];

        return totalPressure;
    };

    std::function<float(void)> getStaticPressurePitot = []()
    {
        float staticPressure;

        auto* hilTransceiver = static_cast<HILConfig::MainHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iPitot = getSampleCounter(sensorData->pitot.NDATA);

        staticPressure = sensorData->pitot.staticPressure[iPitot];

        return staticPressure;
    };

    MockChamberSensor* chamber = nullptr;
    Boardcore::Pitot* pitot    = nullptr;
    bool enableHw;
};
}  // namespace HILTest