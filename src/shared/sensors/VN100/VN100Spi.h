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

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "VN100Data.h"

namespace Boardcore
{

/**
 * @brief Driver class for VN100 Spi IMU.
 */
class VN100Spi : public Sensor<VN100Data>
{
public:
    /**
     * @brief VN100 constructor.
     *
     * @param bus SPI bus.
     * @param csPin SPI chip select pin.
     * @param busConfiguration SPI bus configuration.
     */
    VN100Spi(SPIBus& bus, miosix::GpioPin csPin, SPIBusConfig busConfiguration);

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
     * @brief Gather data from the sensor.
     */
    VN100Data sampleImpl() override;

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
     */
    void sendDummyPacket();
    // TODO: check if really needed and if there is a better solution

    /**
     * @brief Get accelerometer, gyroscope, magnetometer, pressure and
     * temperature measurements from the sensor.
     *
     * @param data The variable where measurements will be stored.
     *
     * @return True if the operation is successful, false otherwise.
     */
    bool getImuSample(VN100Data& data);

    /**
     * @brief Get quaternion measurements from the sensor.
     *
     * @param data The variable where measurements will be stored.
     *
     * @return True if the operation is successful, false otherwise.
     */
    bool getQuaternionSample(VN100Data& data);

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
     * See the datasheet for error codes details.
     */
    uint8_t readRegister(const uint32_t REG_ID, uint8_t* payloadBuf,
                         const uint32_t PAYLOAD_SIZE);

    /**
     * @brief Utility function used to write data to a register of the sensor.
     *
     * @param REG_ID The id of the register to be written.
     * @param payloadBuf The buffer containing the data to be written.
     * @param PAYLOAD_SIZE The amount of data (in bytes) to be written.
     *
     * @return Zero if the operation is successful, the error code otherwise.
     * See the datasheet for error codes details.
     */
    uint8_t writeRegister(const uint32_t REG_ID, uint8_t* payloadBuf,
                          const uint32_t PAYLOAD_SIZE);

    bool isInit = false;

    SPISlave spiSlave;

    PrintLogger logger =
        Logging::getLogger("vn100-spi");  // TODO: is it fine? Should it be
                                          // changed to the exact model number?
};

}  // namespace Boardcore
