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

BMX160::BMX160(SPIBusInterface& bus, miosix::GpioPin cs, BMX160Config config)
    : BMX160(bus, cs, config, SPIBusConfig{})
{
    spi_slave.config.clockDivider = SPI::ClockDivider::DIV_32;
    old_mag.mag_timestamp         = 0.0f;
    old_gyr.gyro_timestamp        = 0.0f;
    old_acc.accel_timestamp       = 0.0f;
}

BMX160::BMX160(SPIBusInterface& bus, miosix::GpioPin cs, BMX160Config config,
               SPIBusConfig bus_config)
    : spi_slave(bus, cs, bus_config), config(config)
{
    old_mag.mag_timestamp   = 0.0f;
    old_gyr.gyro_timestamp  = 0.0f;
    old_acc.accel_timestamp = 0.0f;
}

bool BMX160::init()
{
#ifdef DEBUG
    assert(!is_init && "init() should be called once");
#endif

    if (!checkChipid())
    {
        LOG_ERR(logger, "Got bad CHIPID");
        last_error = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    softReset();

    if (!setPowerMode())
    {

        LOG_ERR(logger, "Not all interfaces are up and running!");
        last_error = SensorErrors::INIT_FAIL;
        return false;
    }

    initAcc();
    initGyr();
    initMag();

    initFifo();
    initInt();

    return is_init = true;
}

bool BMX160::selfTest()
{
#ifdef DEBUG
    assert(is_init && "init() was not called");
#endif

    // The device will enter in an unusable state when testing.
    is_init = false;

    if (!testAcc() || !testGyr() || !testMag())
    {
        last_error = SensorErrors::SELF_TEST_FAIL;
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
    if (irq_enabled)
    {
        SensorFIFO::IRQupdateTimestamp(ts);
    }
}

BMX160Data BMX160::sampleImpl()
{
#ifdef DEBUG
    assert(is_init && "init() was not called");
#endif
    // Reset any errors.
    last_error = SensorErrors::NO_ERRORS;

    // Read temperature
    if (config.temp_divider != 0 && temp_counter % config.temp_divider == 0)
        readTemp();

    temp_counter++;

    // Delete old samples
    last_fifo_level = 0;

    switch (config.fifo_mode)
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

    if (last_error != SensorErrors::NO_ERRORS || last_fifo_level == 0)
    {
        // Something went wrong, return dummy data
        return BMX160Data{};
    }
    else
    {
        return last_fifo[last_fifo_level - 1];
    }
}

BMX160Temperature BMX160::getTemperature()
{
    BMX160Temperature t;
    t.temp_timestamp = TimestampTimer::getTimestamp();
    t.temp           = temperature;
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
    last_fifo[last_fifo_level++] = sample;
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
    SPITransaction spi(spi_slave);
    auto chip_id = spi.readRegister(BMX160Defs::REG_CHIPID);

    return chip_id == BMX160Defs::CHIPID;
}

void BMX160::softReset()
{
    SPITransaction spi(spi_slave);

    // Reset the state of the device, just to be sure.
    sendCmd(spi, BMX160Defs::Cmd::SOFTRESET);
    miosix::Thread::sleep(10);

    // Dummy read of REG_COMM_TEST to enable SPI
    spi.readRegister(BMX160Defs::REG_COMM_TEST);
    miosix::Thread::sleep(10);
}

bool BMX160::setPowerMode()
{
    SPITransaction spi(spi_slave);

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
    switch (config.acc_range)
    {
        case BMX160Config::AccelerometerRange::G_2:
            acc_sensibility = 1.0f / 16384.0f;
            break;
        case BMX160Config::AccelerometerRange::G_4:
            acc_sensibility = 1.0f / 8192.0f;
            break;
        case BMX160Config::AccelerometerRange::G_8:
            acc_sensibility = 1.0f / 4096.0f;
            break;
        case BMX160Config::AccelerometerRange::G_16:
            acc_sensibility = 1.0f / 2048.0f;
            break;
    }

    SPITransaction spi(spi_slave);

    spi.writeRegister(BMX160Defs::REG_ACC_CONF,
                      static_cast<uint8_t>(config.acc_odr) |
                          static_cast<uint8_t>(config.acc_bwp));
    spi.writeRegister(BMX160Defs::REG_ACC_RANGE,
                      static_cast<uint8_t>(config.acc_range));
}

void BMX160::initGyr()
{
    // Calculate gyro sensibility
    switch (config.gyr_range)
    {
        case BMX160Config::GyroscopeRange::DEG_2000:
            gyr_sensibility = 1.0f / 16.4f;
            break;
        case BMX160Config::GyroscopeRange::DEG_1000:
            gyr_sensibility = 1.0f / 32.8f;
            break;
        case BMX160Config::GyroscopeRange::DEG_500:
            gyr_sensibility = 1.0f / 65.6f;
            break;
        case BMX160Config::GyroscopeRange::DEG_250:
            gyr_sensibility = 1.0f / 131.2f;
            break;
        case BMX160Config::GyroscopeRange::DEG_125:
            gyr_sensibility = 1.0f / 262.4f;
            break;
    }

    SPITransaction spi(spi_slave);
    spi.writeRegister(BMX160Defs::REG_GYR_CONF,
                      static_cast<uint8_t>(config.gyr_odr) |
                          static_cast<uint8_t>(config.gyr_bwp));
    spi.writeRegister(BMX160Defs::REG_GYR_RANGE,
                      static_cast<uint8_t>(config.gyr_range));
}

void BMX160::initMag()
{
    /*
    Little explanation of this:
    The magnetometer is not controlled directly,
    instead we have a secondary controller, BMM150,
    with its own register accessed with REG_MAG_IF_[1-3]
    */

    SPITransaction spi(spi_slave);

    // Enable manual configuration mode
    confMag(spi, BMX160Defs::MAG_IF_0_MANUAL);

    // Put MAG into sleep mode (from suspend mode)
    writeMag(spi, BMX160Defs::MAG_REG_RESET,
             BMX160Defs::MAG_RESET_POWER_CONTROL);

    writeMag(spi, BMX160Defs::MAG_REG_REPXY, config.mag_repxy);
    writeMag(spi, BMX160Defs::MAG_REG_REPZ, config.mag_repz);

    if (config.enable_compensation)
        boschReadTrim(spi);

    // Magic sequence to init it
    writeMag(spi, BMX160Defs::MAG_REG_CONTROL, BMX160Defs::MAG_CONTROL_FORCED);
    mapMag(spi, BMX160Defs::MAG_REG_DATA);

    // Set mag output data rate
    spi.writeRegister(BMX160Defs::REG_MAG_CONF,
                      static_cast<uint8_t>(config.mag_odr));
    miosix::Thread::sleep(10);

    // Disable manual configuration mode
    confMag(spi, 0);
}

void BMX160::initFifo()
{
    if (config.fifo_mode != BMX160Config::FifoMode::DISABLED)
    {
        SPITransaction spi(spi_slave);

        uint8_t config_byte = BMX160Defs::FIFO_CONFIG_1_ACC_EN |
                              BMX160Defs::FIFO_CONFIG_1_GYR_EN |
                              BMX160Defs::FIFO_CONFIG_1_MAG_EN;

        if (config.fifo_mode == BMX160Config::FifoMode::HEADER)
            config_byte |= BMX160Defs::FIFO_CONFIG_1_HEADER_EN;

        spi.writeRegister(BMX160Defs::REG_FIFO_CONFIG_1, config_byte);

        config_byte =
            (config.fifo_gyr_downs & 3) | ((config.fifo_acc_downs & 3) << 4);

        if (config.fifo_acc_filtered)
            config_byte |= BMX160Defs::FIFO_DOWNS_ACC_FILT_DATA;

        if (config.fifo_gyr_filtered)
            config_byte |= BMX160Defs::FIFO_DOWNS_GYR_FILT_DATA;

        spi.writeRegister(BMX160Defs::REG_FIFO_DOWNS, config_byte);

        sendCmd(spi, BMX160Defs::Cmd::FIFO_FLUSH);
    }
}

void BMX160::initInt()
{
    if (config.fifo_int != BMX160Config::FifoInterruptPin::DISABLED)
    {
        SPITransaction spi(spi_slave);

        // Select mode between PUSH_PULL or OPEN_DRAIN
        uint8_t out_ctrl = 0;
        out_ctrl |= BMX160Defs::INT_OUT_CTRL_INT2_OUT_EN |
                    BMX160Defs::INT_OUT_CTRL_INT1_OUT_EN;

        if (config.int1_mode == BMX160Config::IntMode::OPEN_DRAIN)
        {
            out_ctrl |= BMX160Defs::INT_OUT_CTRL_INT1_OD;
        }
        if (config.int2_mode == BMX160Config::IntMode::OPEN_DRAIN)
        {
            out_ctrl |= BMX160Defs::INT_OUT_CTRL_INT2_OD;
        }

        // Enable both interrupt pins, otherwise they'll just float.
        // We configure both of them as push-pull and active-low
        spi.writeRegister(BMX160Defs::REG_INT_OUT_CTRL, out_ctrl);

        // Set fifo watermark
        spi.writeRegister(BMX160Defs::REG_FIFO_CONFIG_0, config.fifo_watermark);

        // Enable FIFO full interrupt and fifo watermark
        spi.writeRegister(BMX160Defs::REG_INT_EN_1,
                          BMX160Defs::INT_EN_1_FIFO_FULL |
                              BMX160Defs::INT_EN_1_FIFO_WATERMARK);

        // Enable interrupt pin map
        if (config.fifo_int == BMX160Config::FifoInterruptPin::PIN_INT1)
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

    SPITransaction spi(spi_slave);

    // The acc will complain otherwise...
    spi.writeRegister(BMX160Defs::REG_ACC_CONF, ACC_CONF_TEST);
    spi.writeRegister(BMX160Defs::REG_ACC_RANGE, ACC_RANGE_TEST);

    // Enable acc self-test + positive force + self-test deflection
    spi.writeRegister(BMX160Defs::REG_SELF_TEST,
                      BMX160Defs::SELF_TEST_ACC_AMP |
                          BMX160Defs::SELF_TEST_ACC_SIGN |
                          BMX160Defs::SELF_TEST_ACC_ENABLE);
    miosix::Thread::sleep(50);

    int16_t pos_acc[3];
    spi.readRegisters(BMX160Defs::REG_DATA_ACC,
                      reinterpret_cast<uint8_t*>(pos_acc), sizeof(pos_acc));

    // Enable acc self-test + negative force + self-test deflection
    spi.writeRegister(
        BMX160Defs::REG_SELF_TEST,
        BMX160Defs::SELF_TEST_ACC_AMP | BMX160Defs::SELF_TEST_ACC_ENABLE);
    miosix::Thread::sleep(50);

    int16_t neg_acc[3];
    spi.readRegisters(BMX160Defs::REG_DATA_ACC,
                      reinterpret_cast<uint8_t*>(neg_acc), sizeof(neg_acc));

    if ((neg_acc[0] - pos_acc[0]) < SELF_TEST_LIMIT ||
        (neg_acc[1] - pos_acc[1]) < SELF_TEST_LIMIT ||
        (neg_acc[2] - pos_acc[2]) < SELF_TEST_LIMIT)
    {
        LOG_ERR(logger, "Accelerometer self-test failed!");
        LOG_ERR(logger, "pos_acc: {} {} {}", pos_acc[0], pos_acc[1],
                pos_acc[2]);
        LOG_ERR(logger, "neg_acc: {} {} {}", neg_acc[0], neg_acc[1],
                neg_acc[2]);

        return false;
    }

    // Reset self-test
    spi.writeRegister(BMX160Defs::REG_SELF_TEST, 0);
    return true;
}

bool BMX160::testGyr()
{
    // Start gyro self-test
    SPITransaction spi(spi_slave);

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
    SPITransaction spi(spi_slave);

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

    if (config.enable_compensation)
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
    return AccelerometerData{timestamp,
                             data.x * acc_sensibility * EARTH_GRAVITY,
                             data.y * acc_sensibility * EARTH_GRAVITY,
                             data.z * acc_sensibility * EARTH_GRAVITY};
}

GyroscopeData BMX160::buildGyrData(BMX160Defs::GyrRaw data, uint64_t timestamp)
{
    if (config.gyr_unit == BMX160Config::GyroscopeMeasureUnit::DEG)
    {
        return GyroscopeData{timestamp, data.x * gyr_sensibility,
                             data.y * gyr_sensibility,
                             data.z * gyr_sensibility};
    }
    else
    {
        return GyroscopeData{timestamp,
                             data.x * gyr_sensibility * DEGREES_TO_RADIANS,
                             data.y * gyr_sensibility * DEGREES_TO_RADIANS,
                             data.z * gyr_sensibility * DEGREES_TO_RADIANS};
    }
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
    uint8_t real_odr = static_cast<uint64_t>(odr) - (downs & 3);

    // Hz = 100 / 2^(8-odr)
    // Sec = 2^(13-odr) / 3200
    // Micro = (2^(13-odr)) * 10000 / 32;

    return ((1 << (13 - real_odr)) * 10000) >> 5;
}

void BMX160::readTemp()
{
    SPITransaction spi(spi_slave);

    int16_t val = spi.readRegister(BMX160Defs::REG_TEMPERATURE_0) |
                  (spi.readRegister(BMX160Defs::REG_TEMPERATURE_1) << 8);

    // Correct for sensibility and offset
    temperature = (val * BMX160Defs::TEMP_SENSIBILITY) + 23.0f;
}

void BMX160::readData()
{
    SPITransaction spi(spi_slave);

    uint8_t buf[20];
    spi.readRegisters(BMX160Defs::REG_DATA, buf, sizeof(buf));

    int idx      = 0;
    auto mag_raw = parseStruct<BMX160Defs::MagRaw>(buf, idx);
    auto gyr_raw = parseStruct<BMX160Defs::GyrRaw>(buf, idx);
    auto acc_raw = parseStruct<BMX160Defs::AccRaw>(buf, idx);

    // Push a new sample into the fifo
    pushSample(BMX160Data{
        buildAccData(acc_raw, last_interrupt_us),
        buildGyrData(gyr_raw, last_interrupt_us),
        buildMagData(mag_raw, last_interrupt_us),
    });
}

void BMX160::readFifo(bool headerless)
{
    irq_enabled = false;

    SPITransaction spi(spi_slave);

    int len = spi.readRegister(BMX160Defs::REG_FIFO_LENGTH_0) |
              ((spi.readRegister(BMX160Defs::REG_FIFO_LENGTH_1) & 7) << 8);

    if (len == 0)
    {
        // The buffer is empty, return early
        last_error = SensorErrors::NO_NEW_DATA;
        return;
    }

    uint8_t buf[FIFO_BUF_SIZE];

#ifdef DEBUG
    assert(len <= static_cast<int>(sizeof(buf)) && "Buffer overflow!");
#endif

    // Shift the old timestamps, this allows to use old timestamps with the
    // current frame.
    if (old_mag.mag_timestamp != 0)
        old_mag.mag_timestamp -= dt_interrupt;
    if (old_gyr.gyro_timestamp != 0)
        old_gyr.gyro_timestamp -= dt_interrupt;
    if (old_acc.accel_timestamp != 0)
        old_acc.accel_timestamp -= dt_interrupt;

    // Calculate time offset
    uint64_t time_offset = std::min({
        odrToTimeOffset(config.mag_odr, 0),
        odrToTimeOffset(config.gyr_odr, config.fifo_gyr_downs),
        odrToTimeOffset(config.acc_odr, config.fifo_acc_downs),
    });

    spi.readRegisters(BMX160Defs::REG_FIFO_DATA, buf, len);
    uint64_t timestamp    = 0;
    uint64_t watermark_ts = 0;

    int idx = 0;
    while (idx < len && buf[idx] != BMX160Defs::FIFO_STOP_BYTE)
    {

        if (headerless)
        {
            auto mag_raw = parseStruct<BMX160Defs::MagRaw>(buf, idx);
            auto gyr_raw = parseStruct<BMX160Defs::GyrRaw>(buf, idx);
            auto acc_raw = parseStruct<BMX160Defs::AccRaw>(buf, idx);

            old_mag = buildMagData(mag_raw, timestamp);
            old_gyr = buildGyrData(gyr_raw, timestamp);
            old_acc = buildAccData(acc_raw, timestamp);

            // Push a new sample into the fifo
            pushSample(BMX160Data{old_acc, old_gyr, old_mag});

            if (watermark_ts == 0 && idx >= (config.fifo_watermark * 4))
            {
                watermark_ts = timestamp;
            }

            timestamp += time_offset;
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
                    auto mag_raw = parseStruct<BMX160Defs::MagRaw>(buf, idx);
                    old_mag      = buildMagData(mag_raw, timestamp);
                }

                // This contains gyro data
                if (header & BMX160Defs::FIFO_HEADER_PARM_GYR_DATA)
                {
                    auto gyr_raw = parseStruct<BMX160Defs::GyrRaw>(buf, idx);
                    old_gyr      = buildGyrData(gyr_raw, timestamp);
                }

                // This contains accel data
                if (header & BMX160Defs::FIFO_HEADER_PARM_ACC_DATA)
                {
                    auto acc_raw = parseStruct<BMX160Defs::AccRaw>(buf, idx);
                    old_acc      = buildAccData(acc_raw, timestamp);
                }

                // Push a new sample into the fifo
                pushSample(BMX160Data{old_acc, old_gyr, old_mag});

                if (watermark_ts == 0 && idx >= (config.fifo_watermark * 4))
                {
                    watermark_ts = timestamp;
                }

                timestamp += time_offset;
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

                last_error =
                    static_cast<SensorErrors>(BMX160Errors::INVALID_FIFO_DATA);
                break;
            }
        }
    }

    // Update fifo statistics
    stats.timestamp     = TimestampTimer::getTimestamp();
    stats.watermark_ts  = watermark_ts;
    stats.fifo_duration = timestamp;
    stats.interrupt_dt  = dt_interrupt;
    stats.len           = len;

    // Adjust timestamps
    for (int i = 0; i < last_fifo_level; i++)
    {
        last_fifo[i].accel_timestamp += last_interrupt_us - watermark_ts;
        last_fifo[i].gyro_timestamp += last_interrupt_us - watermark_ts;
        last_fifo[i].mag_timestamp += last_interrupt_us - watermark_ts;
    }

    irq_enabled = true;
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
    uint8_t trim_x1y1[2]     = {0};
    uint8_t trim_xyz_data[4] = {0};
    uint8_t trim_xy1xy2[10]  = {0};

    readMag(spi, BMX160Defs::MAG_REG_DIG_X1, trim_x1y1, sizeof(trim_x1y1));
    readMag(spi, BMX160Defs::MAG_REG_DIG_Z4_0, trim_xyz_data,
            sizeof(trim_xyz_data));
    readMag(spi, BMX160Defs::MAG_REG_DIG_Z2_0, trim_xy1xy2,
            sizeof(trim_xy1xy2));

    // Read trim registers
    trim_data.dig_x1   = trim_x1y1[0];
    trim_data.dig_y1   = trim_x1y1[1];
    trim_data.dig_x2   = trim_xyz_data[2];
    trim_data.dig_y2   = trim_xyz_data[3];
    trim_data.dig_z1   = trim_xy1xy2[2] | (trim_xy1xy2[3] << 8);
    trim_data.dig_z2   = trim_xy1xy2[0] | (trim_xy1xy2[1] << 8);
    trim_data.dig_z3   = trim_xy1xy2[6] | (trim_xy1xy2[7] << 8);
    trim_data.dig_z4   = trim_xyz_data[0] | (trim_xyz_data[1] << 8);
    trim_data.dig_xy1  = trim_xy1xy2[9];
    trim_data.dig_xy2  = trim_xy1xy2[8];
    trim_data.dig_xyz1 = trim_xy1xy2[4] | ((trim_xy1xy2[5] & 0x7F) << 8);
}

float BMX160::boschMagCompensateX(int16_t x, uint16_t rhall)
{
    /* clang-format off */
        float retval = 0;
        float process_comp_x0;
        float process_comp_x1;
        float process_comp_x2;
        float process_comp_x3;
        float process_comp_x4;

        /* Processing compensation equations */
        process_comp_x0 = (((float)trim_data.dig_xyz1) * 16384.0f / rhall);
        retval = (process_comp_x0 - 16384.0f);
        process_comp_x1 = ((float)trim_data.dig_xy2) * (retval * retval / 268435456.0f);
        process_comp_x2 = process_comp_x1 + retval * ((float)trim_data.dig_xy1) / 16384.0f;
        process_comp_x3 = ((float)trim_data.dig_x2) + 160.0f;
        process_comp_x4 = x * ((process_comp_x2 + 256.0f) * process_comp_x3);
        retval = ((process_comp_x4 / 8192.0f) + (((float)trim_data.dig_x1) * 8.0f)) / 16.0f;

        return retval;
    /* clang-format on */
}

float BMX160::boschMagCompensateY(int16_t y, uint16_t rhall)
{
    /* clang-format off */
        float retval = 0;
        float process_comp_y0;
        float process_comp_y1;
        float process_comp_y2;
        float process_comp_y3;
        float process_comp_y4;

        /* Processing compensation equations */
        process_comp_y0 = ((float)trim_data.dig_xyz1) * 16384.0f / rhall;
        retval = process_comp_y0 - 16384.0f;
        process_comp_y1 = ((float)trim_data.dig_xy2) * (retval * retval / 268435456.0f);
        process_comp_y2 = process_comp_y1 + retval * ((float)trim_data.dig_xy1) / 16384.0f;
        process_comp_y3 = ((float)trim_data.dig_y2) + 160.0f;
        process_comp_y4 = y * (((process_comp_y2) + 256.0f) * process_comp_y3);
        retval = ((process_comp_y4 / 8192.0f) + (((float)trim_data.dig_y1) * 8.0f)) / 16.0f;

        return retval;
    /* clang-format on */
}

float BMX160::boschMagCompensateZ(int16_t z, uint16_t rhall)
{
    /* clang-format off */
        float retval = 0;
        float process_comp_z0;
        float process_comp_z1;
        float process_comp_z2;
        float process_comp_z3;
        float process_comp_z4;
        float process_comp_z5;

        process_comp_z0 = ((float)z) - ((float)trim_data.dig_z4);
        process_comp_z1 = ((float)rhall) - ((float)trim_data.dig_xyz1);
        process_comp_z2 = (((float)trim_data.dig_z3) * process_comp_z1);
        process_comp_z3 = ((float)trim_data.dig_z1) * ((float)rhall) / 32768.0f;
        process_comp_z4 = ((float)trim_data.dig_z2) + process_comp_z3;
        process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
        retval = (process_comp_z5 / ((process_comp_z4)*4.0f)) / 16.0f;

        return retval;
    /* clang-format on */
}