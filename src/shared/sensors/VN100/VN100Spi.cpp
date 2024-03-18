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

#include "VN100Spi.h"

#include <drivers/timer/TimestampTimer.h>
#include <interfaces/endianness.h>
#include <utils/Debug.h>

#include "VN100SpiDefs.h"

namespace Boardcore
{

VN100Spi::VN100Spi(SPIBus& bus, miosix::GpioPin csPin,
                   SPIBusConfig busConfiguration)
    : spiSlave(bus, csPin, busConfiguration)
{
}

bool VN100Spi::init()
{
    if (isInit)
    {
        LOG_ERR(logger, "init() should be called once");
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }

    // First communication after startup might fail
    // Send dummy data to clean up
    sendDummyPacket();

    if (checkModelNumber() == false)
    {
        LOG_ERR(logger, "Got bad CHIPID");
        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    isInit    = true;
    lastError = SensorErrors::NO_ERRORS;
    return true;
}

bool VN100Spi::checkModelNumber()
{
    // Ensure that the cast operation from uint8_t to char is legal
    static_assert(sizeof(uint8_t) == sizeof(char) &&
                  "Error, data size mismatch");
    static_assert(alignof(uint8_t) == alignof(char) &&
                  "Error, data alignment mismatch");

    int i = 0;
    char buf[VN100SpiDefs::MODEL_NUMBER_SIZE];
    for (i = 0; i < VN100SpiDefs::MODEL_NUMBER_SIZE; ++i)
    {
        buf[i] = 0;
    }

    if (readRegister(VN100SpiDefs::REG_MODEL_NUMBER, (uint8_t*)buf,
                     VN100SpiDefs::MODEL_NUMBER_SIZE) != 0)
    {
        // An error occurred while attempting to service the request
        LOG_ERR(logger, "Error while reading CHIPID");
        return false;
    }

    // Check the model number
    if (strncmp(VN100SpiDefs::MODEL_NUMBER, buf,
                strlen(VN100SpiDefs::MODEL_NUMBER)) != 0)
    {
        LOG_ERR(logger, "Error, invalid CHIPID");
        return false;
    }

    return true;
}

void VN100Spi::sendDummyPacket()
{
    SPITransaction transaction(spiSlave);
    transaction.write32(0);
}

bool VN100Spi::selfTest()
{
    D(assert(isInit && "init() was not called"));

    if (!checkModelNumber())
    {
        lastError = SensorErrors::SELF_TEST_FAIL;
        return false;
    }

    return true;
}

VN100Data VN100Spi::sampleImpl()
{
    D(assert(isInit && "init() was not called"));

    // Reset any errors.
    lastError = SensorErrors::NO_ERRORS;

    VN100Data data;
    data.accelerationTimestamp  = TimestampTimer::getTimestamp();
    data.angularSpeedTimestamp  = data.accelerationTimestamp;
    data.magneticFieldTimestamp = data.accelerationTimestamp;
    data.pressureTimestamp      = data.accelerationTimestamp;
    data.quaternionTimestamp    = data.accelerationTimestamp;
    data.temperatureTimestamp   = data.accelerationTimestamp;

    if (!getImuSample(data))
    {
        // An error occurred while gathering data
        lastError = NO_NEW_DATA;
        return lastSample;
    }

    if (!getQuaternionSample(data))
    {
        // An error occurred while gathering data
        lastError = NO_NEW_DATA;
        return lastSample;
    }

    return data;
}

bool VN100Spi::getImuSample(VN100Data& data)
{
    uint8_t buf[VN100SpiDefs::IMU_SAMPLE_SIZE];

    if (readRegister(VN100SpiDefs::REG_IMU_DATA, buf,
                     VN100SpiDefs::IMU_SAMPLE_SIZE) != 0)
    {
        // An error occurred while reading data
        return false;
    }

    // Get measurements from raw data
    uint32_t* ptr       = (uint32_t*)buf;
    data.magneticFieldX = extractMeasurement(ptr[0]);
    data.magneticFieldY = extractMeasurement(ptr[1]);
    data.magneticFieldZ = extractMeasurement(ptr[2]);
    data.accelerationX  = extractMeasurement(ptr[3]);
    data.accelerationY  = extractMeasurement(ptr[4]);
    data.accelerationZ  = extractMeasurement(ptr[5]);
    data.angularSpeedX  = extractMeasurement(ptr[6]);
    data.angularSpeedY  = extractMeasurement(ptr[7]);
    data.angularSpeedZ  = extractMeasurement(ptr[8]);
    data.temperature    = extractMeasurement(ptr[9]);
    data.pressure       = extractMeasurement(ptr[10]);

    return true;
}

bool VN100Spi::getQuaternionSample(VN100Data& data)
{
    uint8_t buf[VN100SpiDefs::QUATERNION_SAMPLE_SIZE];

    if (readRegister(VN100SpiDefs::REG_QUATERNION_DATA, buf,
                     VN100SpiDefs::QUATERNION_SAMPLE_SIZE) != 0)
    {
        // An error occurred while reading data
        return false;
    }

    // Get measurements from raw data
    uint32_t* ptr    = (uint32_t*)buf;
    data.quaternionX = extractMeasurement(ptr[0]);
    data.quaternionY = extractMeasurement(ptr[1]);
    data.quaternionZ = extractMeasurement(ptr[2]);
    data.quaternionW = extractMeasurement(ptr[3]);

    return true;
}

float VN100Spi::extractMeasurement(uint32_t rawData)
{
    // The floating point values received are stored as 32-bit IEEE
    // floating point numbers in little endian byte order.

    // Ensure that the copy operation from uint32_t to float is legal
    static_assert(sizeof(uint32_t) == sizeof(float) &&
                  "Error, data size mismatch");

    float f;
    std::memcpy(&f, &rawData, sizeof(uint32_t));

    return f;
}

uint8_t VN100Spi::readRegister(const uint32_t REG_ID, uint8_t* payloadBuf,
                               const uint32_t PAYLOAD_SIZE)
{
    /**
     * When reading from a sensor's register 2 spi transactions are needed.
     *
     * First I have to send the request packet, then wait at least 100
     * microseconds to let the sensor process the request.
     *
     * After this period of time we can proceed with the reading. First
     * we receive a 4 bytes header, whit the first byte always 0, the second
     * being the read register command, the third being the register we asked
     * for and the fourth the error value.
     *
     * Finally we receive the content of the register.
     *
     * Low level spi is needed in order to issue multiple readings without
     * raising the chip select.
     */

    const uint32_t requestPacket =
        (VN100SpiDefs::READ_REG << 24) |  // Read register command
        (REG_ID << 16);                   // Id of the register

    // Send request packet
    spiSlave.bus.select(spiSlave.cs);
    spiSlave.bus.write32(requestPacket);
    spiSlave.bus.deselect(spiSlave.cs);

    // Wait at least 100us
    miosix::delayUs(100);

    // Read response
    spiSlave.bus.select(spiSlave.cs);

    // Discard the first 3 bytes of the response
    spiSlave.bus
        .read24();  // TODO: should I verify also the command and register?

    uint8_t err = spiSlave.bus.read();

    if (err != 0)
    {
        // An error occurred while attempting to service the request
        spiSlave.bus.deselect(spiSlave.cs);
        return err;
    }

    spiSlave.bus.read(payloadBuf, PAYLOAD_SIZE);

    spiSlave.bus.deselect(spiSlave.cs);

    return 0;
}

}  // namespace Boardcore
