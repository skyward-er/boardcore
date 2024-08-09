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

namespace Boardcore
{

VN100Spi::VN100Spi(SPIBus& bus, miosix::GpioPin csPin,
                   SPIBusConfig busConfiguration, uint16_t syncOutSkipFactor)
    : spiSlave(bus, csPin, busConfiguration),
      syncOutSkipFactor(syncOutSkipFactor)
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

    if (!checkModelNumber())
    {
        LOG_ERR(logger, "Got bad CHIPID");
        lastError = SensorErrors::INVALID_WHOAMI;
        return false;
    }

    if (!setInterrupt())
    {
        LOG_ERR(logger, "Unable to set data ready interrupt");
        lastError = SensorErrors::INIT_FAIL;
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
                     VN100SpiDefs::MODEL_NUMBER_SIZE) !=
        VN100SpiDefs::VNErrors::NO_ERROR)
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

    /**
     * After issuing a command the vn100 needs al least 100 microseconds
     * before providing a reply. Considering this function's purpose is
     * to clean the communication I wait a full millisecond before proceeding.
     */
    miosix::Thread::sleep(1);
}

bool VN100Spi::setInterrupt()
{
    /**
     * The data ready interrupt is set via the synchronization control register,
     * by setting the SyncOut mode.
     *
     * Imu data is sampled at 800Hz, while Attitude data (quaternion) is sampled
     * at 400 Hz. Considering that attitude data has a lower rate we set the
     * data ready to trigger on attitude data.
     *
     * We can set the SyncOutSkipFactor, that defines how many times the sync
     * out event should be skipped before actually triggering the SyncOut pin.
     * This way we can control the rate at which the data ready interrupt is
     * triggered.
     *
     * The values not needed for the data ready of the register will be set to
     * default.
     */

    // Init struct and set default values
    VN100SpiDefs::SynchronizationData sData;
    sData.syncInMode = 3;  // Set to: count number of trigger events on SYNC_IN.
    sData.syncInEdge = 0;  // Trigger on rising edge
    sData.syncInSkipFactor = 0;  // Don't skip

    // Set needed values
    sData.syncOutMode = 3;  // Trigger when attitude measurements are available
    sData.syncOutPolarity   = 1;  // Positive output pulse on the SyncOut pin
    sData.syncOutSkipFactor = syncOutSkipFactor;
    sData.syncOutPulseWidth = VN100SpiDefs::SYNC_OUT_PULSE_WIDTH;

    VN100SpiDefs::VNErrors err =
        writeRegister(VN100SpiDefs::REG_SYNC, (uint8_t*)&sData,
                      sizeof(VN100SpiDefs::SynchronizationData));

    if (err != VN100SpiDefs::VNErrors::NO_ERROR)
    {
        TRACE("setInterrupt() failed, error: %u\n", err);
        return false;
    }

    return true;
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

VN100SpiData VN100Spi::sampleImpl()
{
    D(assert(isInit && "init() was not called"));

    // Reset any errors.
    lastError = SensorErrors::NO_ERRORS;

    VN100SpiData data;
    data.accelerationTimestamp  = TimestampTimer::getTimestamp();
    data.angularSpeedTimestamp  = data.accelerationTimestamp;
    data.magneticFieldTimestamp = data.accelerationTimestamp;
    data.quaternionTimestamp    = data.accelerationTimestamp;

    if (!getSample(data))
    {
        // An error occurred while gathering data
        lastError = NO_NEW_DATA;
        return lastSample;
    }

    return data;
}

bool VN100Spi::getSample(VN100SpiData& data)
{
    uint8_t buf[VN100SpiDefs::SAMPLE_SIZE];

    VN100SpiDefs::VNErrors err = readRegister(VN100SpiDefs::REG_QUAT_IMU_DATA,
                                              buf, VN100SpiDefs::SAMPLE_SIZE);

    if (err != VN100SpiDefs::VNErrors::NO_ERROR)
    {
        // An error occurred while reading data
        TRACE("getSample() failed, error: %u\n", err);
        return false;
    }

    // Get measurements from raw data
    uint32_t* ptr       = (uint32_t*)buf;
    data.quaternionX    = extractMeasurement(ptr[0]);
    data.quaternionY    = extractMeasurement(ptr[1]);
    data.quaternionZ    = extractMeasurement(ptr[2]);
    data.quaternionW    = extractMeasurement(ptr[3]);
    data.magneticFieldX = extractMeasurement(ptr[4]);
    data.magneticFieldY = extractMeasurement(ptr[5]);
    data.magneticFieldZ = extractMeasurement(ptr[6]);
    data.accelerationX  = extractMeasurement(ptr[7]);
    data.accelerationY  = extractMeasurement(ptr[8]);
    data.accelerationZ  = extractMeasurement(ptr[9]);
    data.angularSpeedX  = extractMeasurement(ptr[10]);
    data.angularSpeedY  = extractMeasurement(ptr[11]);
    data.angularSpeedZ  = extractMeasurement(ptr[12]);

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

VN100SpiDefs::VNErrors VN100Spi::readRegister(const uint32_t REG_ID,
                                              uint8_t* payloadBuf,
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
    spiSlave.bus.read24();

    VN100SpiDefs::VNErrors err = (VN100SpiDefs::VNErrors)spiSlave.bus.read();

    if (err != VN100SpiDefs::VNErrors::NO_ERROR)
    {
        // An error occurred while attempting to service the request
        spiSlave.bus.deselect(spiSlave.cs);
        return err;
    }

    spiSlave.bus.read(payloadBuf, PAYLOAD_SIZE);

    spiSlave.bus.deselect(spiSlave.cs);

    return VN100SpiDefs::VNErrors::NO_ERROR;
}

VN100SpiDefs::VNErrors VN100Spi::writeRegister(const uint32_t REG_ID,
                                               uint8_t* payloadBuf,
                                               const uint32_t PAYLOAD_SIZE)
{
    /**
     * When writing to a sensor's register 2 spi transactions are needed.
     *
     * First I have to send the request packet with the value to be written,
     * then wait at least 100 microseconds to let the sensor process the
     * request.
     *
     * After this period of time we proceed with reading the outcome of the
     * operation. We receive a 4 bytes header, whit the first byte always 0, the
     * second being the write register command, the third being the register we
     * asked for and the fourth the error value. If the error value is 0 no
     * error occurred and the operation is successful.
     *
     * Low level spi is needed in order to issue multiple readings and writings
     * without raising the chip select.
     */

    const uint32_t requestPacket =
        (VN100SpiDefs::WRITE_REG << 24) |  // Read register command
        (REG_ID << 16);                    // Id of the register

    // Send request packet
    spiSlave.bus.select(spiSlave.cs);
    spiSlave.bus.write32(requestPacket);
    spiSlave.bus.write(payloadBuf, PAYLOAD_SIZE);
    spiSlave.bus.deselect(spiSlave.cs);

    // Wait at least 100us
    miosix::delayUs(100);

    // Read response
    spiSlave.bus.select(spiSlave.cs);

    // Discard the first 3 bytes of the response
    spiSlave.bus.read24();

    uint8_t err = spiSlave.bus.read();

    spiSlave.bus.deselect(spiSlave.cs);

    return (VN100SpiDefs::VNErrors)err;
}

}  // namespace Boardcore
