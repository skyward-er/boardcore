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

#include "Sensors.h"

#include <interfaces-impl/hwmapping.h>

#include "../Buses.h"
#include "SensorsConfig.h"

namespace HILTest
{
using namespace Boardcore;
using namespace std;
using namespace SensorsConfig;

LPS22DFData Sensors::getLPS22DFLastSample()
{
    miosix::PauseKernelLock lock;
    return lps22df != nullptr ? lps22df->getLastSample() : LPS22DFData{};
}
LPS28DFWData Sensors::getLPS28DFW_1LastSample()
{
    miosix::PauseKernelLock lock;
    return lps28dfw_1 != nullptr ? lps28dfw_1->getLastSample() : LPS28DFWData{};
}
LPS28DFWData Sensors::getLPS28DFW_2LastSample()
{
    miosix::PauseKernelLock lock;
    return lps28dfw_2 != nullptr ? lps28dfw_2->getLastSample() : LPS28DFWData{};
}
H3LIS331DLData Sensors::getH3LIS331DLLastSample()
{
    miosix::PauseKernelLock lock;
    return h3lis331dl != nullptr ? h3lis331dl->getLastSample()
                                 : H3LIS331DLData{};
}
LIS2MDLData Sensors::getLIS2MDLLastSample()
{
    miosix::PauseKernelLock lock;
    return lis2mdl != nullptr ? lis2mdl->getLastSample() : LIS2MDLData{};
}
UBXGPSData Sensors::getGPSLastSample()
{
    miosix::PauseKernelLock lock;
    return ubxgps != nullptr ? ubxgps->getLastSample() : UBXGPSData{};
}
LSM6DSRXData Sensors::getLSM6DSRXLastSample()
{
    miosix::PauseKernelLock lock;
    return lsm6dsrx != nullptr ? lsm6dsrx->getLastSample() : LSM6DSRXData{};
}
ADS131M08Data Sensors::getADS131M08LastSample()
{
    miosix::PauseKernelLock lock;
    return ads131m08 != nullptr ? ads131m08->getLastSample() : ADS131M08Data{};
}

PitotData Sensors::getPitotLastSample()
{
    miosix::PauseKernelLock lock;
    return canPitot;
}
PressureData Sensors::getCCPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return canCCPressure;
}
PressureData Sensors::getBottomTankPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return canBottomTankPressure;
}
PressureData Sensors::getTopTankPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return canTopTankPressure;
}
TemperatureData Sensors::getTankTemperatureLastSample()
{
    miosix::PauseKernelLock lock;
    return canTankTemperature;
}
BatteryVoltageSensorData Sensors::getMotorBatteryVoltage()
{
    miosix::PauseKernelLock lock;
    return canMotorBatteryVoltage;
}
CurrentData Sensors::getMotorCurrent()
{
    miosix::PauseKernelLock lock;
    return canMotorCurrent;
}

// Processed Getters
BatteryVoltageSensorData Sensors::getBatteryVoltageLastSample()
{
    // Do not need to pause the kernel, the last sample getter is already
    // protected
    ADS131M08Data sample = getADS131M08LastSample();
    BatteryVoltageSensorData data;

    // Populate the data
    data.voltageTimestamp = sample.timestamp;
    data.channelId        = 1;
    data.voltage          = sample.voltage[1];
    data.batVoltage = sample.voltage[1] * BATTERY_VOLTAGE_CONVERSION_FACTOR;
    return data;
}

BatteryVoltageSensorData Sensors::getCamBatteryVoltageLastSample()
{
    // Do not need to pause the kernel, the last sample getter is already
    // protected
    ADS131M08Data sample = getADS131M08LastSample();
    BatteryVoltageSensorData data;

    // Populate the data
    data.voltageTimestamp = sample.timestamp;
    data.channelId        = 7;
    data.voltage          = sample.voltage[7];
    data.batVoltage = sample.voltage[7] * BATTERY_VOLTAGE_CONVERSION_FACTOR;
    return data;
}

CurrentData Sensors::getCurrentLastSample()
{
    // Do not need to pause the kernel, the last sample getter is already
    // protected
    ADS131M08Data sample = getADS131M08LastSample();
    CurrentData data;

    // Populate the data
    data.currentTimestamp = sample.timestamp;
    data.current =
        (sample.voltage[5] - CURRENT_OFFSET) * CURRENT_CONVERSION_FACTOR;
    return data;
}

MPXH6400AData Sensors::getDeploymentPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return mpxh6400a != nullptr ? mpxh6400a->getLastSample() : MPXH6400AData{};
}

HSCMRNN015PAData Sensors::getStaticPressure1LastSample()
{
    miosix::PauseKernelLock lock;
    return hscmrnn015pa_1 != nullptr ? hscmrnn015pa_1->getLastSample()
                                     : HSCMRNN015PAData{};
}

HSCMRNN015PAData Sensors::getStaticPressure2LastSample()
{
    miosix::PauseKernelLock lock;
    return hscmrnn015pa_2 != nullptr ? hscmrnn015pa_2->getLastSample()
                                     : HSCMRNN015PAData{};
}

RotatedIMUData Sensors::getIMULastSample()
{
    miosix::PauseKernelLock lock;
    return imu != nullptr ? imu->getLastSample() : RotatedIMUData{};
}

MagnetometerData Sensors::getCalibratedMagnetometerLastSample()
{
    // Do not need to pause the kernel, the last sample getter is already
    // protected
    MagnetometerData lastSample = getLIS2MDLLastSample();
    MagnetometerData result;

    // Correct the result and copy the timestamp
    {
        miosix::Lock<FastMutex> l(calibrationMutex);
        result =
            static_cast<MagnetometerData>(magCalibration.correct(lastSample));
    }

    result.magneticFieldTimestamp = lastSample.magneticFieldTimestamp;
    return result;
}

void Sensors::setPitot(PitotData data)
{
    miosix::PauseKernelLock lock;
    canPitot           = data;
    canPitot.timestamp = TimestampTimer::getTimestamp();
}
void Sensors::setCCPressure(PressureData data)
{
    miosix::PauseKernelLock lock;
    canCCPressure                   = data;
    canCCPressure.pressureTimestamp = TimestampTimer::getTimestamp();
}
void Sensors::setBottomTankPressure(PressureData data)
{
    miosix::PauseKernelLock lock;
    canBottomTankPressure                   = data;
    canBottomTankPressure.pressureTimestamp = TimestampTimer::getTimestamp();
}
void Sensors::setTopTankPressure(PressureData data)
{
    miosix::PauseKernelLock lock;
    canTopTankPressure                   = data;
    canTopTankPressure.pressureTimestamp = TimestampTimer::getTimestamp();
}
void Sensors::setTankTemperature(TemperatureData data)
{
    miosix::PauseKernelLock lock;
    canTankTemperature                      = data;
    canTankTemperature.temperatureTimestamp = TimestampTimer::getTimestamp();
}
void Sensors::setMotorBatteryVoltage(BatteryVoltageSensorData data)
{
    miosix::PauseKernelLock lock;
    canMotorBatteryVoltage.batVoltage       = data.batVoltage;
    canMotorBatteryVoltage.voltageTimestamp = TimestampTimer::getTimestamp();
}
void Sensors::setMotorCurrent(CurrentData data)
{
    miosix::PauseKernelLock lock;
    canMotorCurrent.current          = data.current;
    canMotorCurrent.currentTimestamp = TimestampTimer::getTimestamp();
}

Sensors::Sensors(TaskScheduler* sched, Buses* buses)
    : scheduler(sched), buses(buses)
{
    // Init all the sensors
    lps22dfCreation();
    lps28dfw_1Creation();
    // lps28dfw_2Creation();
    h3lis331dlCreation();
    lis2mdlCreation();
    ubxgpsCreation();
    lsm6dsrxCreation();
    ads131m08Creation();
    deploymentPressureCreation();
    staticPressure1Creation();
    staticPressure2Creation();
    imuCreation();
}

bool Sensors::start()
{
    // Read the magnetometer calibration from predefined file
    magCalibration.fromFile("/sd/magCalibration.csv");

    if (lps22df)
    {
        registerSensor(lps22df, "LPS22DF", LPS22DF_PERIOD,
                       [this]() { this->lps22dfCallback(); });
        addInfoGetter(lps22df);
    }

    if (lps28dfw_1)
    {
        registerSensor(lps28dfw_1, "LPS28DFW_1", LPS28DFW_PERIOD,
                       [this]() { this->lps28dfw_1Callback(); });
        addInfoGetter(lps28dfw_1);
    }

    if (lps28dfw_2)
    {
        registerSensor(lps28dfw_2, "LPS28DFW_2", LPS28DFW_PERIOD,
                       [this]() { this->lps28dfw_2Callback(); });
        addInfoGetter(lps28dfw_2);
    }

    if (h3lis331dl)
    {
        registerSensor(h3lis331dl, "h3lis331dl", H3LIS331DL_PERIOD,
                       [this]() { this->h3lis331dlCallback(); });
        addInfoGetter(h3lis331dl);
    }

    if (lis2mdl)
    {
        registerSensor(lis2mdl, "lis2mdl", LIS2MDL_PERIOD,
                       [this]() { this->lis2mdlCallback(); });
        addInfoGetter(lis2mdl);
    }

    if (ubxgps)
    {
        registerSensor(ubxgps, "ubxgps", UBXGPS_PERIOD,
                       [this]() { this->ubxgpsCallback(); });
        addInfoGetter(ubxgps);
    }

    if (lsm6dsrx)
    {
        registerSensor(lsm6dsrx, "lsm6dsrx", LSM6DSRX_PERIOD,
                       [this]() { this->lsm6dsrxCallback(); });
        addInfoGetter(lsm6dsrx);
    }

    if (ads131m08)
    {
        registerSensor(ads131m08, "ads131m08", ADS131M08_PERIOD,
                       [this]() { this->ads131m08Callback(); });
        addInfoGetter(ads131m08);
    }

    if (mpxh6400a)
    {
        registerSensor(mpxh6400a, "mpxh6400a", ADS131M08_PERIOD,
                       [this]() { this->deploymentPressureCallback(); });
    }

    if (hscmrnn015pa_1)
    {
        registerSensor(hscmrnn015pa_1, "hscmrnn015pa_1", ADS131M08_PERIOD,
                       [this]() { this->staticPressure1Callback(); });
    }

    if (hscmrnn015pa_2)
    {
        registerSensor(hscmrnn015pa_2, "hscmrnn015pa_2", ADS131M08_PERIOD,
                       [this]() { this->staticPressure2Callback(); });
    }

    if (imu)
    {
        registerSensor(imu, "RotatedIMU", IMU_PERIOD,
                       [this]() { this->imuCallback(); });
    }

    // Add the magnetometer calibration to the scheduler
    size_t result = scheduler->addTask(
        [&]()
        {
            // Gather the last sample data
            MagnetometerData lastSample = getLIS2MDLLastSample();

            // Feed the data to the calibrator inside a protected area.
            // Contention is not high and the use of a mutex is suitable to
            // avoid pausing the kernel for this calibration operation
            {
                miosix::Lock<FastMutex> l(calibrationMutex);
                magCalibrator.feed(lastSample);
            }
        },
        MAG_CALIBRATION_PERIOD);

    // Create sensor manager with populated map and configured scheduler
    manager = new SensorManager(sensorMap, scheduler);
    return manager->start() && result != 0;
}

void Sensors::stop() { manager->stop(); }

bool Sensors::isStarted()
{
    return manager->areAllSensorsInitialized() && scheduler->isRunning();
}

void Sensors::calibrate()
{
    // Create the stats to calibrate the barometers
    Stats lps28dfw1Stats;
    // Stats lps28dfw2Stats;
    Stats staticPressure1Stats;
    Stats staticPressure2Stats;
    Stats deploymentPressureStats;

    // Add N samples to the stats
    for (unsigned int i = 0; i < SensorsConfig::CALIBRATION_SAMPLES; i++)
    {
        lps28dfw1Stats.add(getLPS28DFW_1LastSample().pressure);
        // lps28dfw2Stats.add(getLPS28DFW_2LastSample().pressure);
        staticPressure1Stats.add(getStaticPressure1LastSample().pressure);
        staticPressure2Stats.add(getStaticPressure2LastSample().pressure);
        deploymentPressureStats.add(getDeploymentPressureLastSample().pressure);

        // Delay for the expected period
        miosix::Thread::sleep(SensorsConfig::CALIBRATION_PERIOD);
    }

    // Compute the difference between the mean value from LPS28DFW
    float reference = lps28dfw1Stats.getStats().mean;

    hscmrnn015pa_1->updateOffset(staticPressure1Stats.getStats().mean -
                                 reference);
    hscmrnn015pa_2->updateOffset(staticPressure2Stats.getStats().mean -
                                 reference);
    mpxh6400a->updateOffset(deploymentPressureStats.getStats().mean -
                            reference);

    // Log the offsets
    SensorsCalibrationParameter cal{};
    cal.timestamp         = TimestampTimer::getTimestamp();
    cal.offsetStatic1     = staticPressure1Stats.getStats().mean - reference;
    cal.offsetStatic2     = staticPressure2Stats.getStats().mean - reference;
    cal.offsetDeployment  = deploymentPressureStats.getStats().mean - reference;
    cal.referencePressure = reference;

    Logger::getInstance().log(cal);
}

bool Sensors::writeMagCalibration()
{
    // Compute the calibration result in protected area
    {
        miosix::Lock<FastMutex> l(calibrationMutex);
        SixParametersCorrector cal = magCalibrator.computeResult();

        // Check result validity
        if (!isnan(cal.getb()[0]) && !isnan(cal.getb()[1]) &&
            !isnan(cal.getb()[2]) && !isnan(cal.getA()[0]) &&
            !isnan(cal.getA()[1]) && !isnan(cal.getA()[2]))
        {
            magCalibration = cal;

            // Save the calibration to the calibration file
            return magCalibration.toFile("/sd/magCalibration.csv");
        }
        return false;
    }
}

std::array<SensorInfo, SensorsConfig::NUMBER_OF_SENSORS>
Sensors::getSensorInfo()
{
    std::array<SensorInfo, SensorsConfig::NUMBER_OF_SENSORS> sensorState;
    for (size_t i = 0; i < sensorsInit.size(); i++)
    {
        // Check wether the lambda is existent
        if (sensorsInit[i])
        {
            sensorState[i] = sensorsInit[i]();
        }
    }
    return sensorState;
}

void Sensors::lps22dfCreation()
{

    // Get the correct SPI configuration
    SPIBusConfig config = LPS22DF::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Configure the device
    LPS22DF::Config sensorConfig;
    sensorConfig.avg = LPS22DF_AVG;
    sensorConfig.odr = LPS22DF_ODR;

    // Create sensor instance with configured parameters
    lps22df = new LPS22DF(buses->spi3, miosix::sensors::LPS22DF::cs::getPin(),
                          config, sensorConfig);
}
void Sensors::lps28dfw_1Creation()
{

    // Configure the sensor
    LPS28DFW::SensorConfig config{false, LPS28DFW_FSR, LPS28DFW_AVG,
                                  LPS28DFW_ODR, false};

    // Create sensor instance with configured parameters
    lps28dfw_1 = new LPS28DFW(buses->i2c1, config);
}
void Sensors::lps28dfw_2Creation()
{

    // Configure the sensor
    LPS28DFW::SensorConfig config{true, LPS28DFW_FSR, LPS28DFW_AVG,
                                  LPS28DFW_ODR, false};

    // Create sensor instance with configured parameters
    lps28dfw_2 = new LPS28DFW(buses->i2c1, config);
}
void Sensors::h3lis331dlCreation()
{

    // Get the correct SPI configuration
    SPIBusConfig config = H3LIS331DL::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Create sensor instance with configured parameters
    h3lis331dl =
        new H3LIS331DL(buses->spi3, miosix::sensors::H3LIS331DL::cs::getPin(),
                       config, H3LIS331DL_ODR, H3LIS331DL_BDU, H3LIS331DL_FSR);
}
void Sensors::lis2mdlCreation()
{

    // Get the correct SPI configuration
    SPIBusConfig config = LIS2MDL::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Configure the sensor
    LIS2MDL::Config sensorConfig;
    sensorConfig.deviceMode         = LIS2MDL_OPERATIVE_MODE;
    sensorConfig.odr                = LIS2MDL_ODR;
    sensorConfig.temperatureDivider = LIS2MDL_TEMPERATURE_DIVIDER;

    // Create sensor instance with configured parameters
    lis2mdl = new LIS2MDL(buses->spi3, miosix::sensors::LIS2MDL::cs::getPin(),
                          config, sensorConfig);
}

void Sensors::ubxgpsCreation()
{

    // Get the correct SPI configuration
    SPIBusConfig config = UBXGPSSpi::getDefaultSPIConfig();
    config.clockDivider = SPI::ClockDivider::DIV_64;

    // Create sensor instance with configured parameters
    ubxgps = new UBXGPSSpi(buses->spi4, miosix::sensors::GPS::cs::getPin(),
                           config, 5);
}

void Sensors::lsm6dsrxCreation()
{

    // Configure the SPI
    SPIBusConfig config;
    config.clockDivider = SPI::ClockDivider::DIV_32;
    config.mode         = SPI::Mode::MODE_0;

    // Configure the sensor
    LSM6DSRXConfig sensorConfig;
    sensorConfig.bdu = LSM6DSRX_BDU;

    // Accelerometer
    sensorConfig.fsAcc     = LSM6DSRX_ACC_FS;
    sensorConfig.odrAcc    = LSM6DSRX_ACC_ODR;
    sensorConfig.opModeAcc = LSM6DSRX_OPERATING_MODE;

    // Gyroscope
    sensorConfig.fsGyr     = LSM6DSRX_GYR_FS;
    sensorConfig.odrGyr    = LSM6DSRX_GYR_ODR;
    sensorConfig.opModeGyr = LSM6DSRX_OPERATING_MODE;

    // Fifo
    sensorConfig.fifoMode                = LSM6DSRX_FIFO_MODE;
    sensorConfig.fifoTimestampDecimation = LSM6DSRX_FIFO_TIMESTAMP_DECIMATION;
    sensorConfig.fifoTemperatureBdr      = LSM6DSRX_FIFO_TEMPERATURE_BDR;

    // Create sensor instance with configured parameters
    lsm6dsrx =
        new LSM6DSRX(buses->spi1, miosix::sensors::LSM6DSRX::cs::getPin(),
                     config, sensorConfig);
}

void Sensors::ads131m08Creation()
{

    // Configure the SPI
    SPIBusConfig config;
    config.clockDivider = SPI::ClockDivider::DIV_32;

    // Configure the device
    ADS131M08::Config sensorConfig;
    sensorConfig.oversamplingRatio     = ADS131M08_OVERSAMPLING_RATIO;
    sensorConfig.globalChopModeEnabled = ADS131M08_GLOBAL_CHOP_MODE;

    // Create the sensor instance with configured parameters
    ads131m08 =
        new ADS131M08(buses->spi4, miosix::sensors::ADS131::cs::getPin(),
                      config, sensorConfig);
}

void Sensors::deploymentPressureCreation()
{
    // Create the lambda function to get the voltage
    function<ADCData()> getVoltage = [&]()
    {
        // No need to synchronize, the sampling thread is the same
        ADS131M08Data sample = ads131m08->getLastSample();
        return sample.getVoltage(ADC_CH_DPL);
    };

    // Create the sensor instance with created function
    mpxh6400a = new MPXH6400A(getVoltage, ADC_VOLTAGE_RANGE);
}

void Sensors::staticPressure1Creation()
{
    // Create the lambda function to get the voltage
    function<ADCData()> getVoltage = [&]()
    {
        // No need to synchronize, the sampling thread is the same
        ADS131M08Data sample = ads131m08->getLastSample();
        return sample.getVoltage(ADC_CH_STATIC_1);
    };

    // Create the sensor instance with created function
    hscmrnn015pa_1 = new HSCMRNN015PA(getVoltage, ADC_VOLTAGE_RANGE);
}

void Sensors::staticPressure2Creation()
{
    // Create the lambda function to get the voltage
    function<ADCData()> getVoltage = [&]()
    {
        // No need to synchronize, the sampling thread is the same
        ADS131M08Data sample = ads131m08->getLastSample();
        return sample.getVoltage(ADC_CH_STATIC_2);
    };

    // Create the sensor instance with created function
    hscmrnn015pa_2 = new HSCMRNN015PA(getVoltage, ADC_VOLTAGE_RANGE);
}

void Sensors::imuCreation()
{
    // Register the IMU as the fake sensor, passing as parameters the methods to
    // retrieve real data. The sensor is not synchronized, but the sampling
    // thread is always the same.
    imu = new RotatedIMU(
        bind(&LSM6DSRX::getLastSample, lsm6dsrx),
        bind(&Sensors::getCalibratedMagnetometerLastSample, this),
        bind(&LSM6DSRX::getLastSample, lsm6dsrx));

    // Invert the Y axis on the magnetometer
    Eigen::Matrix3f m{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
    imu->addMagTransformation(m);
    imu->addAccTransformation(imu->rotateAroundX(45));
    imu->addGyroTransformation(imu->rotateAroundX(45));
    imu->addMagTransformation(imu->rotateAroundX(45));
}

void Sensors::lps22dfCallback()
{
    LPS22DFData lastSample = lps22df->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lps28dfw_1Callback()
{
    LPS28DFW_1Data lastSample =
        static_cast<LPS28DFW_1Data>(lps28dfw_1->getLastSample());
    Logger::getInstance().log(lastSample);
}
void Sensors::lps28dfw_2Callback()
{
    LPS28DFW_2Data lastSample =
        static_cast<LPS28DFW_2Data>(lps28dfw_2->getLastSample());
    Logger::getInstance().log(lastSample);
}
void Sensors::h3lis331dlCallback()
{
    H3LIS331DLData lastSample = h3lis331dl->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lis2mdlCallback()
{
    LIS2MDLData lastSample = lis2mdl->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::ubxgpsCallback()
{
    UBXGPSData lastSample = ubxgps->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::lsm6dsrxCallback()
{
    auto& fifo        = lsm6dsrx->getLastFifo();
    uint16_t fifoSize = lsm6dsrx->getLastFifoSize();

    // For every instance inside the fifo log the sample
    for (uint16_t i = 0; i < fifoSize; i++)
    {
        Logger::getInstance().log(fifo.at(i));
    }
}
void Sensors::ads131m08Callback()
{
    ADS131M08Data lastSample = ads131m08->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::deploymentPressureCallback()
{
    MPXH6400AData lastSample = mpxh6400a->getLastSample();
    Logger::getInstance().log(lastSample);
}
void Sensors::staticPressure1Callback()
{
    HSCMRNN015PA_1Data lastSample =
        static_cast<HSCMRNN015PA_1Data>(hscmrnn015pa_1->getLastSample());
    Logger::getInstance().log(lastSample);
}
void Sensors::staticPressure2Callback()
{
    HSCMRNN015PA_2Data lastSample =
        static_cast<HSCMRNN015PA_2Data>(hscmrnn015pa_2->getLastSample());
    Logger::getInstance().log(lastSample);
}
void Sensors::imuCallback()
{
    RotatedIMUData lastSample = imu->getLastSample();
    Logger::getInstance().log(lastSample);
}

}  // namespace HILTest