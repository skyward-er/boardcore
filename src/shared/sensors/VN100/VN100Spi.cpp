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

    miosix::delayUs(100);

    if (!setInterrupt())
    {
        LOG_ERR(logger, "Unable to set data ready interrupt");
        lastError = SensorErrors::INIT_FAIL;
        return false;
    }

    // Wait to ensure that enough time has passed before the next operation
    miosix::delayUs(100);

    isInit    = true;
    lastError = SensorErrors::NO_ERRORS;
    return true;
}

bool VN100Spi::checkModelNumber()
{
    int i = 0;
    char buf[VN100SpiDefs::MODEL_NUMBER_SIZE];
    for (i = 0; i < VN100SpiDefs::MODEL_NUMBER_SIZE; ++i)
    {
        buf[i] = 0;
    }

    VN100SpiDefs::VNErrors err =
        readRegister(VN100SpiDefs::REG_MODEL_NUMBER, (uint8_t*)buf,
                     VN100SpiDefs::MODEL_NUMBER_SIZE);
    if (err != VN100SpiDefs::VNErrors::NO_ERROR)
    {
        // An error occurred while attempting to service the request
        LOG_ERR(logger, "Error while reading CHIPID");
        TRACE("Error code: %u\n", err);
        return false;
    }

    // Check the model number
    if (strncmp(VN100SpiDefs::MODEL_NUMBER, buf,
                strlen(VN100SpiDefs::MODEL_NUMBER)) != 0)
    {
        LOG_ERR(logger, "Error, invalid CHIPID");
        TRACE("%s != %s\n", VN100SpiDefs::MODEL_NUMBER, buf);
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
     * before providing a reply. Considering this function is called only
     * during the initialization phase I wait for a full millisecond, as a
     * safety measure.
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
     * SyncIn values, which aren't needed for the data ready of the register,
     * will be set to default.
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

bool VN100Spi::selfTest() { return true; }

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
    VN100SpiDefs::RawImuQuatData rawData;

    VN100SpiDefs::VNErrors err = readRegister(
        VN100SpiDefs::REG_QUAT_IMU_DATA, (uint8_t*)&rawData, sizeof(rawData));

    if (err != VN100SpiDefs::VNErrors::NO_ERROR)
    {
        // An error occurred while reading data
        TRACE("getSample() failed, error: %u\n", err);
        return false;
    }

    // Get measurements from raw data
    data.quaternionX    = rawData.quatX;
    data.quaternionY    = rawData.quatY;
    data.quaternionZ    = rawData.quatZ;
    data.quaternionW    = rawData.quatW;
    data.magneticFieldX = rawData.magX;
    data.magneticFieldY = rawData.magY;
    data.magneticFieldZ = rawData.magZ;
    data.accelerationX  = rawData.accX;
    data.accelerationY  = rawData.accY;
    data.accelerationZ  = rawData.accZ;
    data.angularSpeedX  = rawData.gyrX;
    data.angularSpeedY  = rawData.gyrY;
    data.angularSpeedZ  = rawData.gyrZ;

    return true;
}

TemperatureData VN100Spi::getTemperature()
{
    TemperatureData data;

    VN100SpiDefs::RawTempPressData rawData;

    // Get timestamp
    data.temperatureTimestamp = TimestampTimer::getTimestamp();

    VN100SpiDefs::VNErrors err = readRegister(
        VN100SpiDefs::REG_TEMP_PRESS_DATA, (uint8_t*)&rawData, sizeof(rawData));

    if (err != VN100SpiDefs::VNErrors::NO_ERROR)
    {
        // An error occurred while reading data
        TRACE("getTemperature() failed, error: %u\n", err);
        return data;
    }

    // Get measurement from raw data
    data.temperature = rawData.temp;

    return data;
}

PressureData VN100Spi::getPressure()
{
    PressureData data;

    VN100SpiDefs::RawTempPressData rawData;

    // Get timestamp
    data.pressureTimestamp = TimestampTimer::getTimestamp();

    VN100SpiDefs::VNErrors err = readRegister(
        VN100SpiDefs::REG_TEMP_PRESS_DATA, (uint8_t*)&rawData, sizeof(rawData));

    if (err != VN100SpiDefs::VNErrors::NO_ERROR)
    {
        // An error occurred while reading data
        TRACE("getPressure() failed, error: %u\n", err);
        return data;
    }

    // Get measurement from raw data
    data.pressure = rawData.press;

    return data;
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
    VN100SpiDefs::VNErrors err =
        (VN100SpiDefs::VNErrors)(spiSlave.bus.read32() & 255);

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
                                               const uint8_t* payloadBuf,
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
    uint8_t err = spiSlave.bus.read32() & 255;

    spiSlave.bus.deselect(spiSlave.cs);

    return (VN100SpiDefs::VNErrors)err;
}

}  // namespace Boardcore
