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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include <algorithm>
#include <cstring>

#include "BMX160Config.h"
#include "BMX160Data.h"
#include "BMX160Defs.h"
#include "Constants.h"
#include "TimestampTimer.h"

namespace Boardcore
{

/**
 * @brief BMX160 Driver.
 */
class BMX160 : public SensorFIFO<BMX160Data, 200>
{
public:
    /**
     * @brief BMX160 Custom errors.
     */
    enum BMX160Errors : uint8_t
    {
        INVALID_FIFO_DATA =
            SensorErrors::END_OF_BASE_ERRORS  ///< The fifo contained invalid
                                              ///< data.
    };

    /**
     * @brief BMX160 Constructor.
     * @param bus SPI bus
     * @param cs SPI Chip Select pin
     * @param config BMX160 configuration
     */
    BMX160(SPIBusInterface& bus, GpioPin cs, BMX160Config config = {});

    /**
     * @brief BMX160 Constructor.
     * @param bus SPI bus
     * @param cs SPI Chip Select pin
     * @param config BMX160 configuration
     * @param bus_config SPI bus configuration
     */
    BMX160(SPIBusInterface& bus, GpioPin cs, BMX160Config config,
           SPIBusConfig bus_config);

    /**
     * @brief Initialize the driver.
     */
    bool init() override;

    /**
     * @brief Perform selftest on the device.
     */
    bool selfTest() override;

    /**
     * @brief Gather data from FIFO/data registers and temperature sensor.
     */
    BMX160Data sampleImpl() override;

    /**
     * @brief Get last read temperature.
     */
    BMX160Temperature getTemperature();

    /**
     * @brief Retrieve last fifo stats.
     */
    BMX160FifoStats getFifoStats();

    /**
     * @brief Sometimes the sensor pulls down the interrupt pin while reading
     * data. We override this method and update the timestamps only if we are
     * not still reading the fifo ( @see{irq_enabled} ).
     */
    void IRQupdateTimestamp(uint64_t ts) override;

private:
    /**
     * @brief Execute CMD.
     *
     * @param cmd Command to be executed.
     * @param pmu Powermode (MUST be set to PowerMode::SUSPEND when not needed).
     */
    void sendCmd(SPITransaction& spi, BMX160Defs::Cmd cmd,
                 BMX160Defs::PowerMode pmu = BMX160Defs::PowerMode::SUSPEND);

    /**
     * @brief Push a sample into the FIFO.
     *
     * @param sample Sample to be pushed.
     */
    void pushSample(BMX160Data sample);

    /**
     * @brief Convenience function to configure magnetometer.
     *
     * @param value bmm150 configuration value.
     */
    void confMag(SPITransaction& spi, uint8_t value);

    /**
     * @brief Convenience function to map magnetometer for read.
     *
     * @param reg bmm150 register to be mapped in REG_DATA_MAG.
     */
    void mapMag(SPITransaction& spi, uint8_t reg);

    /**
     * @brief Convenience function to read a single byte from magnetometer.
     *
     * @param reg Register to be read.
     * @return Value read from the register.
     */
    uint8_t readMag(SPITransaction& spi, uint8_t reg);

    /**
     * @brief Convenience function to read from magnetometer.
     *
     * @param reg Register to be read.
     * @param[out] data Buffer to fill with the data.
     * @param size Size of the buffer.
     */
    void readMag(SPITransaction& spi, uint8_t reg, uint8_t* data, size_t size);

    /**
     * @brief Convenience function to write to magnetometer.
     *
     * @param reg Register to be written to.
     * @param value Value to write to the register.
     */
    void writeMag(SPITransaction& spi, uint8_t reg, uint8_t value);

    /**
     * @brief Check for chipid validity.
     *
     * @return Returns false on failure.
     */
    bool checkChipid();

    /**
     * @brief Perform a soft-reset.
     */
    void softReset();

    /**
     * @brief Wake interfaces up, should be called after soft-reset.
     *
     * @return Returns false on failure.
     */
    bool setPowerMode();

    /**
     * @brief Initialize accelerometer.
     */
    void initAcc();

    /**
     * @brief Initialize gyroscope.
     */
    void initGyr();

    /**
     * @brief Initialize magnetometer.
     */
    void initMag();

    /**
     * @brief Initialize FIFO.
     */
    void initFifo();

    /**
     * @brief Initialize interrupts.
     */
    void initInt();

    /**
     * @brief Self-test accelerometer.
     *
     * @return Returns false on failure.
     */
    bool testAcc();

    /**
     * @brief Self-test gyroscope.
     *
     * @return Returns false on failure.
     */
    bool testGyr();

    /**
     * @brief Self-test magnetometer.
     *
     * @return Returns false on failure.
     */
    bool testMag();

    /**
     * @brief Build magnetometer data.
     *
     * @param data Raw input data.
     * @param timestamp Timestamp associated with the data.
     */
    MagnetometerData buildMagData(BMX160Defs::MagRaw data, uint64_t timestamp);

    /**
     * @brief Build gyroscope data.
     *
     * @param data Raw input data.
     * @param timestamp Timestamp associated with the data.
     */
    AccelerometerData buildAccData(BMX160Defs::AccRaw data, uint64_t timestamp);

    /**
     * @brief Build accelerometer data.
     *
     * @param data Raw input data.
     * @param timestamp Timestamp associated with the data.
     */
    GyroscopeData buildGyrData(BMX160Defs::GyrRaw data, uint64_t timestamp);

    /**
     * @brief Debug function used to print the current error state
     *
     * @return String representing the error.
     */
    const char* debugErr(SPITransaction& spi);

    /**
     * @brief Convert Output Data Rate to the time between samples.
     *
     * Warning, anything resulting in a frequency over 1Hz will underflow the
     * calculations!
     *
     * @param odr input output data rate of the sensor.
     * @param downs downsampling of the input.
     * @return Time between samples.
     */
    uint64_t odrToTimeOffset(BMX160Config::OutputDataRate odr, uint8_t downs);

    /**
     * @brief Read the value of the temperature sensor
     */
    void readTemp();

    /**
     * @brief Read the contents of the DATA register
     */
    void readData();

    /**
     * @brief Read the contents of the fifo into buf.
     *
     * @return Returns false on failure.
     */
    void readFifo(bool headerless);

    /**
     * @brief Parses T out of a buffer.
     *
     * @param buf Input buffer.
     * @param[in,out] idx Input buffer read index.
     */
    template <typename T>
    T parseStruct(uint8_t* buf, int& idx);

    /**========================================================================
     * Warning: the following code is extrapolated from the bosch driver
     * source code, I have no idea of what it does.
     * https://github.com/BoschSensortec/BMM150-Sensor-API/blob/a20641f216057f0c54de115fe81b57368e119c01/bmm150.c#L1614
     * ========================================================================
     */

    /**
     * @brief Read and parse bosch trim registers
     */
    void boschReadTrim(SPITransaction& spi);

    /**
     * @brief Bosch black-box algorithm
     */
    float boschMagCompensateX(int16_t x, uint16_t rhall);

    /**
     * @brief Bosch black-box algorithm
     */
    float boschMagCompensateY(int16_t y, uint16_t rhall);

    /**
     * @brief Bosch black-box algorithm
     */
    float boschMagCompensateZ(int16_t z, uint16_t rhall);

    float temperature = 0.0f;
    MagnetometerData old_mag;
    GyroscopeData old_gyr;
    AccelerometerData old_acc;

    bool is_init = false;
    SPISlave spi_slave;

    BMX160Defs::TrimData trim_data;
    BMX160Config config;

    BMX160FifoStats stats;

    float gyr_sensibility = 0.0f;
    float acc_sensibility = 0.0f;

    int temp_counter = 0;

    // Sometimes the buffer gets over 1000
    static constexpr unsigned int FIFO_BUF_SIZE = 1100;

    /**
     * Sometimes the sensor pulls down the interrupt pin while reading data.
     * We use this variable to ignore incoming interrupts while reading the
     * fifo.
     */
    bool irq_enabled = true;

    PrintLogger logger = Logging::getLogger("bmx160");
};

}  // namespace Boardcore
