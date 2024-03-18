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
    int i = 0;
    char buf[VN100SpiDefs::MODEL_NUMBER_SIZE];
    for (i = 0; i < VN100SpiDefs::MODEL_NUMBER_SIZE; ++i)
    {
        buf[i] = 0;
    }

    constexpr uint32_t requestPacket =
        (VN100SpiDefs::READ_REG << 24) |         // Read register command
        (VN100SpiDefs::REG_MODEL_NUMBER << 16);  // Id of the register

    // Send request packet
    {
        SPITransaction transaction{spiSlave};

        transaction.write32(requestPacket);
    }

    // Wait at least 100us
    miosix::delayUs(100);

    // Read response
    uint8_t err = 0;
    {
        // Low level spi is needed in order to read multiple data
        // without raising the chip select pin

        spiSlave.bus.select(spiSlave.cs);

        // Discard the first 3 bytes of the response
        spiSlave.bus
            .read24();  // TODO: should I verify also the command and register?

        err = spiSlave.bus.read();

        spiSlave.bus.read((uint8_t*)buf, VN100SpiDefs::MODEL_NUMBER_SIZE);

        spiSlave.bus.deselect(spiSlave.cs);
    }

    if (err != 0)
    {
        // An error occurred while attempting to service the request
        LOG_ERR(logger, "Error, cannot get CHIPID");
        return false;
    }

    // Check the model number
    return strncmp(VN100SpiDefs::MODEL_NUMBER, buf,
                   strlen(VN100SpiDefs::MODEL_NUMBER)) == 0;
}

bool VN100Spi::selfTest()
{
    // TODO

    return true;
}

VN100Data VN100Spi::sampleImpl()
{
    // TODO

    return VN100Data();
}

AccelerometerData VN100Spi::readAcc()
{
    // Low level spi is needed in order to read multiple data
    // without raising the chip select pin

    AccelerometerData data;

    // Variables used to store raw data extracted from the sensor
    uint32_t rawDataX = 0;
    uint32_t rawDataY = 0;
    uint32_t rawDataZ = 0;

    constexpr uint32_t requestPacket =
        (VN100SpiDefs::READ_REG << 24) |  // Read register command
        (18 << 16);                       // Id of the register

    data.accelerationTimestamp = TimestampTimer::getTimestamp();

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
        lastError = COMMAND_FAILED;
        return AccelerometerData();
    }

    rawDataX = spiSlave.bus.read32();
    rawDataY = spiSlave.bus.read32();
    rawDataZ = spiSlave.bus.read32();

    spiSlave.bus.deselect(spiSlave.cs);

    // Get measurements from raw data
    data.accelerationX = extractMeasurement(rawDataX);
    data.accelerationY = extractMeasurement(rawDataY);
    data.accelerationZ = extractMeasurement(rawDataZ);

    return data;
}

float VN100Spi::extractMeasurement(uint32_t rawData)
{
    // The floating point values received are stored as 32-bit IEEE
    // floating point numbers in little endian byte order.

    // Ensure that the operation is legal
    static_assert(sizeof(uint32_t) == sizeof(float) &&
                  "Error, data size mismatch");

    rawData = swapBytes32(rawData);

    float f;
    std::memcpy(&f, &rawData, sizeof(uint32_t));

    return f;
}

}  // namespace Boardcore
