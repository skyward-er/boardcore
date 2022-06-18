/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "BMX160.h"

#include <assert.h>
#include <utils/Constants.h>

namespace Boardcore
{

BMX160::BMX160(SPIBusInterface& bus, miosix::GpioPin cs, BMX160Config config)
    : BMX160(bus, cs, config, SPIBusConfig{})
{
    spiSlave.config.clockDivider    = SPI::ClockDivider::DIV_32;
    oldMag.magneticFieldTimestamp   = 0.0f;
    oldGyr.angularVelocityTimestamp = 0.0f;
    oldAcc.accelerationTimestamp    = 0.0f;
}

BMX160::BMX160(SPIBusInterface& bus, miosix::GpioPin cs, BMX160Config config,
               SPIBusConfig bus_config)
    : spiSlave(bus, cs, bus_config), config(config)
{
    oldMag.magneticFieldTimestamp   = 0.0f;
    oldGyr.angularVelocityTimestamp = 0.0f;
    oldAcc.accelerationTimestamp    = 0.0f;
}

bool BMX160::init()
{
#ifdef DEBUG
    assert(!isInit && "init() should be called once");
#endif

    if (!checkChipid())
    {
        LOG_ERR(logger, "Got bad CHIPID");
        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    softReset();

    if (!setPowerMode())
    {

        LOG_ERR(logger, "Not all interfaces are up and running!");
        lastError = SensorErrors::INIT_FAIL;
        return false;
    }

    initAcc();
    initGyr();
    initMag();

    initFifo();
    initInt();

    return isInit = true;
}

bool BMX160::selfTest()
{
#ifdef DEBUG
    assert(isInit && "init() was not called");  // linter off
#endif

    // The device will enter in an unusable state when testing.
    isInit = false;

    if (!testAcc() || !testGyr() || !testMag())
    {
        lastError = SensorErrors::SELF_TEST_FAIL;
        return false;
    }
    else
    {
        // Finally reinitialize the device into a known state
        return init();
    }
}

void BMX160::IRQupdateTimestamp(uint64_t ts)
{
    // Prevent interrupts while reading fifo
    if (irqEnabled)
    {
        SensorFIFO::IRQupdateTimestamp(ts);
    }
}

BMX160Data BMX160::sampleImpl()
{
#ifdef DEBUG
    assert(isInit && "init() was not called");
#endif
    // Reset any errors.
    lastError = SensorErrors::NO_ERRORS;

    // Read temperature
    if (config.temperatureDivider != 0 &&
        tempCounter % config.temperatureDivider == 0)
        readTemp();

    tempCounter++;

    // Delete old samples
    lastFifoLevel = 0;

    switch (config.fifoMode)
    {
        case BMX160Config::FifoMode::DISABLED:
            // Just push one sample
            readData();
            break;

        case BMX160Config::FifoMode::HEADERLESS:
            // Read whole FIFO (headerless)
            readFifo(true);
            break;

        case BMX160Config::FifoMode::HEADER:
            // Read whole FIFO (header)
            readFifo(false);
            break;
    }

    if (lastError != SensorErrors::NO_ERRORS || lastFifoLevel == 0)
    {
        // Something went wrong, return dummy data
        return BMX160Data{};
    }
    else
    {
        return lastFifo[lastFifoLevel - 1];
    }
}

BMX160Temperature BMX160::getTemperature()
{
    BMX160Temperature t;
    t.temperatureTimestamp = TimestampTimer::getTimestamp();
    t.temperature          = temperature;
    return t;
}

BMX160FifoStats BMX160::getFifoStats() { return stats; }

void BMX160::sendCmd(SPITransaction& spi, BMX160Defs::Cmd cmd,
                     BMX160Defs::PowerMode pmu)
{
    spi.writeRegister(BMX160Defs::REG_CMD,
                      static_cast<uint8_t>(cmd) | static_cast<uint8_t>(pmu));
}

void BMX160::pushSample(BMX160Data sample)
{
    lastFifo[lastFifoLevel++] = sample;
}

void BMX160::confMag(SPITransaction& spi, uint8_t value)
{
    spi.writeRegister(BMX160Defs::REG_MAG_IF_0, value);
    miosix::Thread::sleep(10);
}

void BMX160::mapMag(SPITransaction& spi, uint8_t reg)
{
    spi.writeRegister(BMX160Defs::REG_MAG_IF_1, reg);
    miosix::Thread::sleep(10);
}

uint8_t BMX160::readMag(SPITransaction& spi, uint8_t reg)
{
    mapMag(spi, reg);
    return spi.readRegister(BMX160Defs::REG_DATA_MAG);
}

void BMX160::readMag(SPITransaction& spi, uint8_t reg, uint8_t* data,
                     size_t size)
{
    while (size != 0)
    {
        int burst = 0;
        if (size >= 8)
        {
            confMag(spi,
                    BMX160Defs::MAG_IF_0_MANUAL | BMX160Defs::MAG_IF_0_BURST_8);
            burst = 8;
        }
        else if (size >= 6)
        {
            confMag(spi,
                    BMX160Defs::MAG_IF_0_MANUAL | BMX160Defs::MAG_IF_0_BURST_6);
            burst = 6;
        }
        else if (size >= 2)
        {
            confMag(spi,
                    BMX160Defs::MAG_IF_0_MANUAL | BMX160Defs::MAG_IF_0_BURST_2);
            burst = 2;
        }
        else
        {
            confMag(spi,
                    BMX160Defs::MAG_IF_0_MANUAL | BMX160Defs::MAG_IF_0_BURST_1);
            burst = 1;
        }

        mapMag(spi, reg);
        spi.readRegisters(BMX160Defs::REG_DATA_MAG, data, burst);

        reg += burst;
        data += burst;
        size -= burst;
    }

    confMag(spi, BMX160Defs::MAG_IF_0_MANUAL);
}

void BMX160::writeMag(SPITransaction& spi, uint8_t reg, uint8_t value)
{
    spi.writeRegister(BMX160Defs::REG_MAG_IF_3, value);
    spi.writeRegister(BMX160Defs::REG_MAG_IF_2, reg);
    miosix::Thread::sleep(10);
}

bool BMX160::checkChipid()
{
    SPITransaction spi(spiSlave);
    auto chipId = spi.readRegister(BMX160Defs::REG_CHIPID);

    return chipId == BMX160Defs::CHIPID;
}

void BMX160::softReset()
{
    SPITransaction spi(spiSlave);

    // Reset the state of the device, just to be sure.
    sendCmd(spi, BMX160Defs::Cmd::SOFTRESET);
    miosix::Thread::sleep(10);

    // Dummy read of REG_COMM_TEST to enable SPI
    spi.readRegister(BMX160Defs::REG_COMM_TEST);
    miosix::Thread::sleep(10);
}

bool BMX160::setPowerMode()
{
    SPITransaction spi(spiSlave);

    sendCmd(spi, BMX160Defs::Cmd::MAG_IF_SET_PMU_MODE,
            BMX160Defs::PowerMode::NORMAL);
    miosix::Thread::sleep(80);

    sendCmd(spi, BMX160Defs::Cmd::GYR_SET_PMU_MODE,
            BMX160Defs::PowerMode::NORMAL);
    miosix::Thread::sleep(80);

    sendCmd(spi, BMX160Defs::Cmd::ACC_SET_PMU_MODE,
            BMX160Defs::PowerMode::NORMAL);
    miosix::Thread::sleep(80);

    // Check if all sensors are up and running
    return (spi.readRegister(BMX160Defs::REG_PMU_STATUS) &
            BMX160Defs::PMU_STATUS_ALL_MASK) ==
           BMX160Defs::PMU_STATUS_ALL_NORMAL;
}

void BMX160::initAcc()
{
    // Calculate accelerometer sensibility
    switch (config.accelerometerRange)
    {
        case BMX160Config::AccelerometerRange::G_2:
            accSensibility = 1.0f / 16384.0f;
            break;
        case BMX160Config::AccelerometerRange::G_4:
            accSensibility = 1.0f / 8192.0f;
            break;
        case BMX160Config::AccelerometerRange::G_8:
            accSensibility = 1.0f / 4096.0f;
            break;
        case BMX160Config::AccelerometerRange::G_16:
            accSensibility = 1.0f / 2048.0f;
            break;
    }

    SPITransaction spi(spiSlave);

    spi.writeRegister(BMX160Defs::REG_ACC_CONF,
                      static_cast<uint8_t>(config.accelerometerDataRate) |
                          static_cast<uint8_t>(config.accelerometerBandwidth));
    spi.writeRegister(BMX160Defs::REG_ACC_RANGE,
                      static_cast<uint8_t>(config.accelerometerRange));
}

void BMX160::initGyr()
{
    // Calculate gyro sensibility
    switch (config.gyroscopeRange)
    {
        case BMX160Config::GyroscopeRange::DEG_2000:
            gyrSensibility = 1.0f / 16.4f;
            break;
        case BMX160Config::GyroscopeRange::DEG_1000:
            gyrSensibility = 1.0f / 32.8f;
            break;
        case BMX160Config::GyroscopeRange::DEG_500:
            gyrSensibility = 1.0f / 65.6f;
            break;
        case BMX160Config::GyroscopeRange::DEG_250:
            gyrSensibility = 1.0f / 131.2f;
            break;
        case BMX160Config::GyroscopeRange::DEG_125:
            gyrSensibility = 1.0f / 262.4f;
            break;
    }

    SPITransaction spi(spiSlave);
    spi.writeRegister(BMX160Defs::REG_GYR_CONF,
                      static_cast<uint8_t>(config.gyroscopeDataRate) |
                          static_cast<uint8_t>(config.gyroscopeBandwidth));
    spi.writeRegister(BMX160Defs::REG_GYR_RANGE,
                      static_cast<uint8_t>(config.gyroscopeRange));
}

void BMX160::initMag()
{
    /*
    Little explanation of this:
    The magnetometer is not controlled directly,
    instead we have a secondary controller, BMM150,
    with its own register accessed with REG_MAG_IF_[1-3]
    */

    SPITransaction spi(spiSlave);

    // Enable manual configuration mode
    confMag(spi, BMX160Defs::MAG_IF_0_MANUAL);

    // Put MAG into sleep mode (from suspend mode)
    writeMag(spi, BMX160Defs::MAG_REG_RESET,
             BMX160Defs::MAG_RESET_POWER_CONTROL);

    writeMag(spi, BMX160Defs::MAG_REG_REPXY, config.magnetometerRepetitionsXY);
    writeMag(spi, BMX160Defs::MAG_REG_REPZ, config.magnetometerRepetitionsZ);

    if (config.enableCompensation)
        boschReadTrim(spi);

    // Magic sequence to init it
    writeMag(spi, BMX160Defs::MAG_REG_CONTROL, BMX160Defs::MAG_CONTROL_FORCED);
    mapMag(spi, BMX160Defs::MAG_REG_DATA);

    // Set mag output data rate
    spi.writeRegister(BMX160Defs::REG_MAG_CONF,
                      static_cast<uint8_t>(config.magnetometerRate));
    miosix::Thread::sleep(10);

    // Disable manual configuration mode
    confMag(spi, 0);
}

void BMX160::initFifo()
{
    if (config.fifoMode != BMX160Config::FifoMode::DISABLED)
    {
        SPITransaction spi(spiSlave);

        uint8_t configByte = BMX160Defs::FIFO_CONFIG_1_ACC_EN |
                             BMX160Defs::FIFO_CONFIG_1_GYR_EN |
                             BMX160Defs::FIFO_CONFIG_1_MAG_EN;

        if (config.fifoMode == BMX160Config::FifoMode::HEADER)
            configByte |= BMX160Defs::FIFO_CONFIG_1_HEADER_EN;

        spi.writeRegister(BMX160Defs::REG_FIFO_CONFIG_1, configByte);

        configByte = (config.fifoGyroscopeDownsampling & 3) |
                     ((config.fifoAccelerationDownsampling & 3) << 4);

        if (config.fifoAccelerometerFiltered)
            configByte |= BMX160Defs::FIFO_DOWNS_ACC_FILT_DATA;

        if (config.fifoGyroscopeFiltered)
            configByte |= BMX160Defs::FIFO_DOWNS_GYR_FILT_DATA;

        spi.writeRegister(BMX160Defs::REG_FIFO_DOWNS, configByte);

        sendCmd(spi, BMX160Defs::Cmd::FIFO_FLUSH);
    }
}

void BMX160::initInt()
{
    if (config.fifoInterrupt != BMX160Config::FifoInterruptPin::DISABLED)
    {
        SPITransaction spi(spiSlave);

        // Select mode between PUSH_PULL or OPEN_DRAIN
        uint8_t outCtrl = 0;
        outCtrl |= BMX160Defs::INT_OUT_CTRL_INT2_OUT_EN |
                   BMX160Defs::INT_OUT_CTRL_INT1_OUT_EN;

        if (config.interrupt1Mode == BMX160Config::IntMode::OPEN_DRAIN)
        {
            outCtrl |= BMX160Defs::INT_OUT_CTRL_INT1_OD;
        }
        if (config.interrupt2Mode == BMX160Config::IntMode::OPEN_DRAIN)
        {
            outCtrl |= BMX160Defs::INT_OUT_CTRL_INT2_OD;
        }

        // Enable both interrupt pins, otherwise they'll just float.
        // We configure both of them as push-pull and active-low
        spi.writeRegister(BMX160Defs::REG_INT_OUT_CTRL, outCtrl);

        // Set fifo watermark
        spi.writeRegister(BMX160Defs::REG_FIFO_CONFIG_0, config.fifoWatermark);

        // Enable FIFO full interrupt and fifo watermark
        spi.writeRegister(BMX160Defs::REG_INT_EN_1,
                          BMX160Defs::INT_EN_1_FIFO_FULL |
                              BMX160Defs::INT_EN_1_FIFO_WATERMARK);

        // Enable interrupt pin map
        if (config.fifoInterrupt == BMX160Config::FifoInterruptPin::PIN_INT1)
        {
            // Configure to use INT1
            spi.writeRegister(BMX160Defs::REG_INT_MAP_1,
                              BMX160Defs::INT_MAP_1_INT_1_FIFO_FULL |
                                  BMX160Defs::INT_MAP_1_INT_1_FIFO_WATERMARK);
        }
        else
        {
            // Configure to use INT2
            spi.writeRegister(BMX160Defs::REG_INT_MAP_1,
                              BMX160Defs::INT_MAP_1_INT_2_FIFO_FULL |
                                  BMX160Defs::INT_MAP_1_INT_2_FIFO_WATERMARK);
        }
    }
}

bool BMX160::testAcc()
{
    const uint16_t SELF_TEST_LIMIT = 8192;
    const uint8_t ACC_CONF_TEST    = 0x2C;
    const uint8_t ACC_RANGE_TEST   = 0x08;

    SPITransaction spi(spiSlave);

    // The acc will complain otherwise...
    spi.writeRegister(BMX160Defs::REG_ACC_CONF, ACC_CONF_TEST);
    spi.writeRegister(BMX160Defs::REG_ACC_RANGE, ACC_RANGE_TEST);

    // Enable acc self-test + positive force + self-test deflection
    spi.writeRegister(BMX160Defs::REG_SELF_TEST,
                      BMX160Defs::SELF_TEST_ACC_AMP |
                          BMX160Defs::SELF_TEST_ACC_SIGN |
                          BMX160Defs::SELF_TEST_ACC_ENABLE);
    miosix::Thread::sleep(50);

    int16_t posAcc[3];
    spi.readRegisters(BMX160Defs::REG_DATA_ACC,
                      reinterpret_cast<uint8_t*>(posAcc), sizeof(posAcc));

    // Enable acc self-test + negative force + self-test deflection
    spi.writeRegister(
        BMX160Defs::REG_SELF_TEST,
        BMX160Defs::SELF_TEST_ACC_AMP | BMX160Defs::SELF_TEST_ACC_ENABLE);
    miosix::Thread::sleep(50);

    int16_t negAcc[3];
    spi.readRegisters(BMX160Defs::REG_DATA_ACC,
                      reinterpret_cast<uint8_t*>(negAcc), sizeof(negAcc));

    if ((negAcc[0] - posAcc[0]) < SELF_TEST_LIMIT ||
        (negAcc[1] - posAcc[1]) < SELF_TEST_LIMIT ||
        (negAcc[2] - posAcc[2]) < SELF_TEST_LIMIT)
    {
        LOG_ERR(logger, "Accelerometer self-test failed!");
        LOG_ERR(logger, "posAcc: {} {} {}", posAcc[0], posAcc[1], posAcc[2]);
        LOG_ERR(logger, "negAcc: {} {} {}", negAcc[0], negAcc[1], negAcc[2]);

        return false;
    }

    // Reset self-test
    spi.writeRegister(BMX160Defs::REG_SELF_TEST, 0);
    return true;
}

bool BMX160::testGyr()
{
    // Start gyro self-test
    SPITransaction spi(spiSlave);

    spi.writeRegister(BMX160Defs::REG_SELF_TEST, BMX160Defs::SELF_TEST_GYR);

    miosix::Thread::sleep(50);

    // Read back the results
    if (!(spi.readRegister(BMX160Defs::REG_STATUS) & 2))
    {
        LOG_ERR(logger, "Gyroscope self-test failed!");
        return false;
    }
    else
    {
        return true;
    }
}

bool BMX160::testMag()
{
    SPITransaction spi(spiSlave);

    // Enable manual configuration mode
    confMag(spi, BMX160Defs::MAG_IF_0_MANUAL);

    // Enable self-test and put magnetometer in sleep
    writeMag(spi, BMX160Defs::MAG_REG_CONTROL,
             BMX160Defs::MAG_CONTROL_SELF_TEST | BMX160Defs::MAG_CONTROL_SLEEP);
    miosix::Thread::sleep(200);

    // Check if it has finished
    if (readMag(spi, BMX160Defs::MAG_REG_CONTROL) &
        BMX160Defs::MAG_CONTROL_SELF_TEST)
    {
        LOG_ERR(logger, "Magnetometer didn't finish self-test!");
        return false;
    }

    // Read back test results
    int16_t mag[4];
    readMag(spi, BMX160Defs::MAG_REG_DATA, reinterpret_cast<uint8_t*>(mag),
            sizeof(mag));

    // Test results are stored in the lower bit of the 3 axis
    if (!(mag[0] & 1) || !(mag[1] & 1) || !(mag[2] & 1))
    {
        LOG_ERR(logger, "Magnetometer self-test failed!");
        LOG_ERR(logger, "result: %d %d %d %d\n", mag[0], mag[1], mag[2],
                mag[3]);
        return false;
    }
    else
    {
        return true;
    }
}

MagnetometerData BMX160::buildMagData(BMX160Defs::MagRaw data,
                                      uint64_t timestamp)
{
    // Strip the lower 3 bits for xy
    data.x >>= 3;
    data.y >>= 3;
    // Strip the lower 1 bit for z
    data.z >>= 1;

    if (config.enableCompensation)
    {
        return MagnetometerData{timestamp,
                                boschMagCompensateX(data.x, data.rhall),
                                boschMagCompensateY(data.y, data.rhall),
                                boschMagCompensateZ(data.z, data.rhall)};
    }
    else
    {
        return MagnetometerData{timestamp, data.x * BMX160Defs::MAG_SENSIBILITY,
                                data.y * BMX160Defs::MAG_SENSIBILITY,
                                data.z * BMX160Defs::MAG_SENSIBILITY};
    }
}

AccelerometerData BMX160::buildAccData(BMX160Defs::AccRaw data,
                                       uint64_t timestamp)
{
    using namespace Constants;

    return AccelerometerData{timestamp, data.x * accSensibility * g,
                             data.y * accSensibility * g,
                             data.z * accSensibility * g};
}

GyroscopeData BMX160::buildGyrData(BMX160Defs::GyrRaw data, uint64_t timestamp)
{
    using namespace Constants;

    if (config.gyroscopeUnit == BMX160Config::GyroscopeMeasureUnit::DEG)
        return GyroscopeData{timestamp, data.x * gyrSensibility,
                             data.y * gyrSensibility, data.z * gyrSensibility};
    else
        return GyroscopeData{timestamp,
                             data.x * gyrSensibility * DEGREES_TO_RADIANS,
                             data.y * gyrSensibility * DEGREES_TO_RADIANS,
                             data.z * gyrSensibility * DEGREES_TO_RADIANS};
}

const char* BMX160::debugErr(SPITransaction& spi)
{
    uint8_t err = spi.readRegister(BMX160Defs::REG_ERR);

    if (err & 1)
    {
        return "Chip not operable";
    }
    else if (err & 64)
    {
        return "Dropped command to register 0x7E";
    }
    else
    {
        // Mask error code
        err = (err >> 1) & 0x0F;
        switch (err)
        {
            case 0:
                return "No error";
            case 1:
            case 2:
                return "Generic error";
            case 3:
                return "LPM and interrupt uses pre-filtered data";
            case 6:
                return "ODR do not match";
            case 7:
                return "LPM uses pre-filtered data";
            default:
                return "Reserved error";
        }
    }
}

uint64_t BMX160::odrToTimeOffset(BMX160Config::OutputDataRate odr,
                                 uint8_t downs)
{
    // Adjust ODR for downsampling
    uint8_t realOdr = static_cast<uint64_t>(odr) - (downs & 3);

    // Hz = 100 / 2^(8-odr)
    // Sec = 2^(13-odr) / 3200
    // Micro = (2^(13-odr)) * 10000 / 32;

    return ((1 << (13 - realOdr)) * 10000) >> 5;
}

void BMX160::readTemp()
{
    SPITransaction spi(spiSlave);

    int16_t val = spi.readRegister(BMX160Defs::REG_TEMPERATURE_0) |
                  (spi.readRegister(BMX160Defs::REG_TEMPERATURE_1) << 8);

    // Correct for sensibility and offset
    temperature = (val * BMX160Defs::TEMP_SENSIBILITY) + 23.0f;
}

void BMX160::readData()
{
    SPITransaction spi(spiSlave);

    uint8_t buf[20];
    spi.readRegisters(BMX160Defs::REG_DATA, buf, sizeof(buf));

    int idx     = 0;
    auto magRaw = parseStruct<BMX160Defs::MagRaw>(buf, idx);
    auto gyrRaw = parseStruct<BMX160Defs::GyrRaw>(buf, idx);
    auto accRaw = parseStruct<BMX160Defs::AccRaw>(buf, idx);

    // Push a new sample into the fifo
    pushSample(BMX160Data{
        buildAccData(accRaw, lastInterruptTimestamp),
        buildGyrData(gyrRaw, lastInterruptTimestamp),
        buildMagData(magRaw, lastInterruptTimestamp),
    });
}

void BMX160::readFifo(bool headerless)
{
    irqEnabled = false;

    SPITransaction spi(spiSlave);

    int len = spi.readRegister(BMX160Defs::REG_FIFO_LENGTH_0) |
              ((spi.readRegister(BMX160Defs::REG_FIFO_LENGTH_1) & 7) << 8);

    if (len == 0)
    {
        // The buffer is empty, return early
        lastError = SensorErrors::NO_NEW_DATA;
        return;
    }

    uint8_t buf[FIFO_BUF_SIZE];

#ifdef DEBUG
    assert(len <= static_cast<int>(sizeof(buf)) && "Buffer overflow!");
#endif

    // Shift the old timestamps, this allows to use old timestamps with the
    // current frame.
    if (oldMag.magneticFieldTimestamp != 0)
        oldMag.magneticFieldTimestamp -= interruptTimestampDelta;
    if (oldGyr.angularVelocityTimestamp != 0)
        oldGyr.angularVelocityTimestamp -= interruptTimestampDelta;
    if (oldAcc.accelerationTimestamp != 0)
        oldAcc.accelerationTimestamp -= interruptTimestampDelta;

    // Calculate time offset
    uint64_t timeOffset = std::min({
        odrToTimeOffset(config.magnetometerRate, 0),
        odrToTimeOffset(config.gyroscopeDataRate,
                        config.fifoGyroscopeDownsampling),
        odrToTimeOffset(config.accelerometerDataRate,
                        config.fifoAccelerationDownsampling),
    });

    spi.readRegisters(BMX160Defs::REG_FIFO_DATA, buf, len);
    uint64_t timestamp          = 0;
    uint64_t watermarkTimestamp = 0;

    int idx = 0;
    while (idx < len && buf[idx] != BMX160Defs::FIFO_STOP_BYTE)
    {

        if (headerless)
        {
            auto magRaw = parseStruct<BMX160Defs::MagRaw>(buf, idx);
            auto gyrRaw = parseStruct<BMX160Defs::GyrRaw>(buf, idx);
            auto accRaw = parseStruct<BMX160Defs::AccRaw>(buf, idx);

            oldMag = buildMagData(magRaw, timestamp);
            oldGyr = buildGyrData(gyrRaw, timestamp);
            oldAcc = buildAccData(accRaw, timestamp);

            // Push a new sample into the fifo
            pushSample(BMX160Data{oldAcc, oldGyr, oldMag});

            if (watermarkTimestamp == 0 && idx >= (config.fifoWatermark * 4))
            {
                watermarkTimestamp = timestamp;
            }

            timestamp += timeOffset;
        }
        else
        {
            uint8_t header = buf[idx++];

            if ((header & BMX160Defs::FIFO_HEADER_MODE_MASK) ==
                BMX160Defs::FIFO_HEADER_MODE_REGULAR)
            {
                // This is a regular packet

                // Mask out everything but fh_parm
                header &= BMX160Defs::FIFO_HEADER_PARM_MASK;

                // This contains magnet data
                if (header & BMX160Defs::FIFO_HEADER_PARM_MAG_DATA)
                {
                    auto magRaw = parseStruct<BMX160Defs::MagRaw>(buf, idx);
                    oldMag      = buildMagData(magRaw, timestamp);
                }

                // This contains gyro data
                if (header & BMX160Defs::FIFO_HEADER_PARM_GYR_DATA)
                {
                    auto gyrRaw = parseStruct<BMX160Defs::GyrRaw>(buf, idx);
                    oldGyr      = buildGyrData(gyrRaw, timestamp);
                }

                // This contains accel data
                if (header & BMX160Defs::FIFO_HEADER_PARM_ACC_DATA)
                {
                    auto accRaw = parseStruct<BMX160Defs::AccRaw>(buf, idx);
                    oldAcc      = buildAccData(accRaw, timestamp);
                }

                // Push a new sample into the fifo
                pushSample(BMX160Data{oldAcc, oldGyr, oldMag});

                if (watermarkTimestamp == 0 &&
                    idx >= (config.fifoWatermark * 4))
                {
                    watermarkTimestamp = timestamp;
                }

                timestamp += timeOffset;
            }
            else if ((header & BMX160Defs::FIFO_HEADER_MODE_MASK) ==
                     BMX160Defs::FIFO_HEADER_MODE_CONTROL)
            {
                // This is a control packet

                // Mask out everything but fh_parm
                header &= BMX160Defs::FIFO_HEADER_PARM_MASK;

                if (header == BMX160Defs::FIFO_HEADER_PARM_SKIP)
                {
                    // Skip frame
                    idx += 1;
                }
                else if (header == BMX160Defs::FIFO_HEADER_PARM_SENSORTIME)
                {
                    // Sensortime frame
                    idx += 3;
                }
                else if (header == BMX160Defs::FIFO_HEADER_PARM_CONFIG)
                {
                    // FIFO_input_config_frame
                    idx += 1;
                }
            }
            else
            {
                LOG_ERR(logger, "Malformed packet! Aborting fifo transfer...");

                lastError =
                    static_cast<SensorErrors>(BMX160Errors::INVALID_FIFO_DATA);
                break;
            }
        }
    }

    // Update fifo statistics
    stats.timestamp               = TimestampTimer::getTimestamp();
    stats.watermarkTimestamp      = watermarkTimestamp;
    stats.fifoDuration            = timestamp;
    stats.interruptTimestampDelta = interruptTimestampDelta;
    stats.len                     = len;

    // Adjust timestamps
    for (int i = 0; i < lastFifoLevel; i++)
    {
        lastFifo[i].accelerationTimestamp +=
            lastInterruptTimestamp - watermarkTimestamp;
        lastFifo[i].angularVelocityTimestamp +=
            lastInterruptTimestamp - watermarkTimestamp;
        lastFifo[i].magneticFieldTimestamp +=
            lastInterruptTimestamp - watermarkTimestamp;
    }

    irqEnabled = true;
}

template <typename T>
T BMX160::parseStruct(uint8_t* buf, int& idx)
{
    T data;
    memcpy(&data, buf + idx, sizeof(T));
    idx += sizeof(T);

    return data;
}

/**========================================================================
 * Warning: the following code is extrapolated from the bosch driver
 * source code, I have no idea of what it does.
 * https://github.com/BoschSensortec/BMM150-Sensor-API/blob/a20641f216057f0c54de115fe81b57368e119c01/bmm150.c#L1614
 * ========================================================================
 */

void BMX160::boschReadTrim(SPITransaction& spi)
{
    uint8_t trimX1y1[2]    = {0};
    uint8_t trimXyzData[4] = {0};
    uint8_t trimXy1xy2[10] = {0};

    readMag(spi, BMX160Defs::MAG_REG_DIG_X1, trimX1y1, sizeof(trimX1y1));
    readMag(spi, BMX160Defs::MAG_REG_DIG_Z4_0, trimXyzData,
            sizeof(trimXyzData));
    readMag(spi, BMX160Defs::MAG_REG_DIG_Z2_0, trimXy1xy2, sizeof(trimXy1xy2));

    // Read trim registers
    trimData.digX1   = trimX1y1[0];
    trimData.digY1   = trimX1y1[1];
    trimData.digX2   = trimXyzData[2];
    trimData.digY2   = trimXyzData[3];
    trimData.digZ1   = trimXy1xy2[2] | (trimXy1xy2[3] << 8);
    trimData.digZ2   = trimXy1xy2[0] | (trimXy1xy2[1] << 8);
    trimData.digZ3   = trimXy1xy2[6] | (trimXy1xy2[7] << 8);
    trimData.digZ4   = trimXyzData[0] | (trimXyzData[1] << 8);
    trimData.digXY1  = trimXy1xy2[9];
    trimData.digXY2  = trimXy1xy2[8];
    trimData.digXYZ1 = trimXy1xy2[4] | ((trimXy1xy2[5] & 0x7F) << 8);
}

float BMX160::boschMagCompensateX(int16_t x, uint16_t rhall)
{
    /* clang-format off */
        float retval = 0;
        float processCompX0;
        float processCompX1;
        float processCompX2;
        float processCompX3;
        float processCompX4;

        /* Processing compensation equations */
        processCompX0 = (((float)trimData.digXYZ1) * 16384.0f / rhall);
        retval = (processCompX0 - 16384.0f);
        processCompX1 = ((float)trimData.digXY2) * (retval * retval / 268435456.0f);
        processCompX2 = processCompX1 + retval * ((float)trimData.digXY1) / 16384.0f;
        processCompX3 = ((float)trimData.digX2) + 160.0f;
        processCompX4 = x * ((processCompX2 + 256.0f) * processCompX3);
        retval = ((processCompX4 / 8192.0f) + (((float)trimData.digX1) * 8.0f)) / 16.0f;

        return retval;
    /* clang-format on */
}

float BMX160::boschMagCompensateY(int16_t y, uint16_t rhall)
{
    /* clang-format off */
        float retval = 0;
        float processCompY0;
        float processCompY1;
        float processCompY2;
        float processCompY3;
        float processCompY4;

        /* Processing compensation equations */
        processCompY0 = ((float)trimData.digXYZ1) * 16384.0f / rhall;
        retval = processCompY0 - 16384.0f;
        processCompY1 = ((float)trimData.digXY2) * (retval * retval / 268435456.0f);
        processCompY2 = processCompY1 + retval * ((float)trimData.digXY1) / 16384.0f;
        processCompY3 = ((float)trimData.digY2) + 160.0f;
        processCompY4 = y * (((processCompY2) + 256.0f) * processCompY3);
        retval = ((processCompY4 / 8192.0f) + (((float)trimData.digY1) * 8.0f)) / 16.0f;

        return retval;
    /* clang-format on */
}

float BMX160::boschMagCompensateZ(int16_t z, uint16_t rhall)
{
    /* clang-format off */
        float retval = 0;
        float processCompZ0;
        float processCompZ1;
        float processCompZ2;
        float processCompZ3;
        float processCompZ4;
        float processCompZ5;

        processCompZ0 = ((float)z) - ((float)trimData.digZ4);
        processCompZ1 = ((float)rhall) - ((float)trimData.digXYZ1);
        processCompZ2 = (((float)trimData.digZ3) * processCompZ1);
        processCompZ3 = ((float)trimData.digZ1) * ((float)rhall) / 32768.0f;
        processCompZ4 = ((float)trimData.digZ2) + processCompZ3;
        processCompZ5 = (processCompZ0 * 131072.0f) - processCompZ2;
        retval = (processCompZ5 / ((processCompZ4)*4.0f)) / 16.0f;

        return retval;
    /* clang-format on */
}

}  // namespace Boardcore
