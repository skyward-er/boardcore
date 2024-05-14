/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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

/**
 * Driver for the VN100 SPI IMU.
 *
 * The VN100 sensor is a calibrated IMU which includes accelerometer,
 * magnetometer, gyroscope, barometer and temperature sensor. It also provides
 * attitude data (yaw, pith, roll, quaternion).
 * This driver samples imu compensated data (accelerometer, gyroscope and
 * magnetometer) and quaternion data.
 *
 * The sampling rate is 400Hz. The data ready interrupt can be set to a lower
 * rate by changing the syncOutSkipFactor parameter.
 *
 * ATTENTION: at least 100 microseconds has to pass between the read/write
 * operations with the sensor.
 */

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "VN100SpiData.h"
#include "VN100SpiDefs.h"

namespace Boardcore
{

/**
 * @brief Driver class for VN100 Spi IMU.
 */
class VN100Spi : public Sensor<VN100SpiData>
{
public:
    /**
     * @brief VN100 constructor.
     *
     * @param bus SPI bus.
     * @param csPin SPI chip select pin.
     * @param busConfiguration SPI bus configuration.
     * @param syncOutSkipFactor The SyncOutSkipFactor defines how many times the
     * data ready event should be skipped before actually triggering the
     * interrupt pin.
     */
    VN100Spi(SPIBus& bus, miosix::GpioPin csPin, SPIBusConfig busConfiguration,
             uint16_t syncOutSkipFactor);

    /**
     * @brief Initialize the sensor.
     */
    bool init() override;

    /**
     * @brief Performs self test for the sensor.
     *
     * @return Return true if the test was successful.
     */
    bool selfTest() override;

    /**
     * @brief Retrieve temperature data from the sensor.
     */
    TemperatureData getTemperature();

    /**
     * @brief Retrieve pressure data from the sensor.
     */
    PressureData getPressure();

protected:
    /**
     * @brief Gather data from the sensor.
     */
    VN100SpiData sampleImpl() override;

private:
    /**
     * @brief Check the model number register.
     *
     * @return Returns false if the red value is not valid.
     */
    bool checkModelNumber();

    /**
     * @brief Utility function used to clean the junk before starting to
     * communicate with the sensor. It send a 4 bytes packet of zeros to the
     * sensor.
     *
     * TODO: this function is used because of a bug with spi mode 3 peripherals,
     * remove it when the bug is fixed.
     */
    void sendDummyPacket();

    /**
     * @brief Set the data ready interrupt.
     *
     * @return True if the operation is successful, false otherwise.
     */
    bool setInterrupt();

    /**
     * @brief Get quaternion, accelerometer, gyroscope and magnetometer
     * measurements from the sensor.
     *
     * @param data The variable where measurements will be stored.
     *
     * @return True if the operation is successful, false otherwise.
     */
    bool getSample(VN100SpiData& data);

    /**
     * @brief Extracts floating point measurement from the data received from
     * the sensor.
     *
     * @param rawData The data received from the sensor.
     *
     * @return The floating point data.
     */
    float extractMeasurement(uint32_t rawData);

    /**
     * @brief Utility function used to read from a register of the sensor.
     *
     * @param REG_ID The id of the register to read from.
     * @param payloadBuf The buffer where data will be stored.
     * @param PAYLOAD_SIZE The amount of data (in bytes) to be read from the
     * register.
     *
     * @return Zero if the operation is successful, the error code otherwise.
     */
    VN100SpiDefs::VNErrors readRegister(const uint32_t REG_ID,
                                        uint8_t* payloadBuf,
                                        const uint32_t PAYLOAD_SIZE);

    /**
     * @brief Utility function used to write data to a register of the sensor.
     *
     * @param REG_ID The id of the register to be written.
     * @param payloadBuf The buffer containing the data to be written.
     * @param PAYLOAD_SIZE The amount of data (in bytes) to be written.
     *
     * @return Zero if the operation is successful, the error code otherwise.
     */
    VN100SpiDefs::VNErrors writeRegister(const uint32_t REG_ID,
                                         const uint8_t* payloadBuf,
                                         const uint32_t PAYLOAD_SIZE);

    bool isInit = false;

    SPISlave spiSlave;

    /**
     * @brief The SyncOutSkipFactor defines how many times the sync out event
     * should be skipped before actually triggering the SyncOut pin (data
     * ready).
     */
    const uint16_t syncOutSkipFactor = 0;

    PrintLogger logger = Logging::getLogger("vn100-spi");
};

}  // namespace Boardcore
