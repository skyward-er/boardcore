/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Davide Bonomini, Davide Mor, Alberto Nidasio
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

#include "UBXGPSSerial.h"

#include <diagnostic/StackLogger.h>
#include <drivers/serial.h>
#include <drivers/timer/TimestampTimer.h>
#include <fcntl.h>
#include <filesystem/file_access.h>

using namespace miosix;

namespace Boardcore
{

UBXGPSSerial::UBXGPSSerial(int baudrate, uint8_t sampleRate, int serialPortNum,
                           const char* serialPortName, int defaultBaudrate)
    : baudrate(baudrate), sampleRate(sampleRate),
      serialPortNumber(serialPortNum), serialPortName(serialPortName),
      defaultBaudrate(defaultBaudrate)
{
    // Prepare the gps file path with the specified name
    strcpy(gpsFilePath, "/dev/");
    strcat(gpsFilePath, serialPortName);
}

bool UBXGPSSerial::init()
{
    LOG_DEBUG(logger, "Changing device baudrate...");

    if (!setSerialCommunication())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Error while setting serial communication");
        return false;
    }

    LOG_DEBUG(logger, "Resetting the device...");

    if (!reset())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not reset the device");
        return false;
    }

    LOG_DEBUG(logger, "Setting the UBX protocol...");

    if (!setUBXProtocol())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the UBX protocol");
        return false;
    }

    LOG_DEBUG(logger, "Setting the dynamic model...");

    if (!setDynamicModelToAirborne4g())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the dynamic model");
        return false;
    }

    LOG_DEBUG(logger, "Setting the sample rate...");

    if (!setSampleRate())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the sample rate");
        return false;
    }

    LOG_DEBUG(logger, "Setting the PVT message rate...");

    if (!setPVTMessageRate())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the PVT message rate");
        return false;
    }

    return true;
}

bool UBXGPSSerial::selfTest() { return true; }

UBXGPSData UBXGPSSerial::sampleImpl()
{
    Lock<FastMutex> l(mutex);
    return threadSample;
}

bool UBXGPSSerial::reset()
{
    uint8_t payload[] = {
        0x00, 0x00,  // Hot start
        0x00,        // Hardware reset
        0x00         // Reserved
    };

    UBXFrame frame{UBXMessage::UBX_CFG_RST, payload, sizeof(payload)};

    // The reset message will not be followed by an acknowledgment
    if (!writeUBXFrame(frame))
        return false;

    // Do not interact with the module while it is resetting
    miosix::Thread::sleep(RESET_SLEEP_TIME);

    return true;
}

bool UBXGPSSerial::setBaudrate()
{
    uint8_t payload[] = {
        0x00,                    // Version
        0xff,                    // All layers
        0x00, 0x00,              // Reserved
        0x01, 0x00, 0x52, 0x40,  // Configuration item key ID
        0xff, 0xff, 0xff, 0xff,  // Value
    };

    // Prepare baudrate
    payload[8]  = baudrate;
    payload[9]  = baudrate >> 8;
    payload[10] = baudrate >> 16;
    payload[11] = baudrate >> 24;

    UBXFrame frame{UBXMessage::UBX_CFG_VALSET, payload, sizeof(payload)};

    return safeWriteUBXFrame(frame);
}

bool UBXGPSSerial::setSerialCommunication()
{
    intrusive_ref_ptr<DevFs> devFs = FilesystemManager::instance().getDevFs();

    // Change the baudrate only if it is different than the default
    if (baudrate != defaultBaudrate)
    {
        // Close the gps file if already opened
        devFs->remove(serialPortName);

        // Open the serial port device with the default boudrate
        if (!devFs->addDevice(serialPortName,
                              intrusive_ref_ptr<Device>(new STM32Serial(
                                  serialPortNumber, defaultBaudrate))))
        {
            LOG_ERR(logger,
                    "[gps] Failed to open serial port {0} with baudrate {1} as "
                    "file {2}",
                    serialPortNumber, defaultBaudrate, serialPortName);
            return false;
        }

        // Open the gps file
        if ((gpsFile = open(gpsFilePath, O_RDWR)) < 0)
        {
            LOG_ERR(logger, "Failed to open gps file {}", gpsFilePath);
            return false;
        }

        // Change boudrate
        if (!setBaudrate())
        {
            return false;
        };

        // Close the gps file
        if (close(gpsFile) < 0)
        {
            LOG_ERR(logger, "Failed to close gps file {}", gpsFilePath);
            return false;
        }

        // Close the serial port
        if (!devFs->remove(serialPortName))
        {
            LOG_ERR(logger, "Failed to close serial port {} as file {}",
                    serialPortNumber, serialPortName);
            return false;
        }
    }

    // Reopen the serial port with the configured boudrate
    if (!devFs->addDevice(serialPortName,
                          intrusive_ref_ptr<Device>(
                              new STM32Serial(serialPortNumber, baudrate))))
    {
        LOG_ERR(logger,
                "Failed to open serial port {} with baudrate {} as file {}\n",
                serialPortNumber, defaultBaudrate, serialPortName);
        return false;
    }

    // Reopen the gps file
    if ((gpsFile = open(gpsFilePath, O_RDWR)) < 0)
    {
        LOG_ERR(logger, "Failed to open gps file {}", gpsFilePath);
        return false;
    }

    return true;
}

bool UBXGPSSerial::setUBXProtocol()
{
    uint8_t payload[] = {
        0x04,                    // SPI port
        0x00,                    // reserved0
        0x00, 0x00,              // txReady
        0x00, 0x00, 0x00, 0x00,  // spiMode = 0, ffCnt = 0 (mechanism off)
        0x00, 0x00, 0x00, 0x00,  // reserved1
        0x01, 0x00,              // inProtoMask = UBX
        0x01, 0x00,              // outProtoMask = UBX
        0x00, 0x00,              // flags
        0x00, 0x00               // reserved2
    };

    UBXFrame frame{UBXMessage::UBX_CFG_PRT, payload, sizeof(payload)};

    return safeWriteUBXFrame(frame);
}

bool UBXGPSSerial::setDynamicModelToAirborne4g()
{
    uint8_t payload[] = {
        0x01, 0x00,  // Parameters bitmask, apply dynamic model configuration
        0x08,        // Dynamic model = airbone 4g
        0x00,        // Fix mode
        0x00, 0x00, 0x00, 0x00,  // Fixed altitude for 2D mode
        0x00, 0x00, 0x00, 0x00,  // Fixed altitude variance for 2D mode
        0x00,        // Minimun elevation for a GNSS satellite to be used
        0x00,        // Reserved
        0x00, 0x00,  // Position DOP mask to use
        0x00, 0x00,  // Time DOP mask to use
        0x00, 0x00,  // Position accuracy mask
        0x00, 0x00,  // Time accuracy mask
        0x00,        // Static hold threshold
        0x00,        // DGNSS timeout
        0x00,        // C/NO threshold number SVs
        0x00,        // C/NO threshold
        0x00, 0x00,  // Reserved
        0x00, 0x00,  // Static hold distance threshold
        0x00,        // UTC standard to be used
        0x00, 0x00, 0x00, 0x00, 0x00  // Reserved
    };

    UBXFrame frame{UBXMessage::UBX_CFG_NAV5, payload, sizeof(payload)};

    return safeWriteUBXFrame(frame);
}

bool UBXGPSSerial::setSampleRate()
{
    uint8_t payload[] = {
        0x00, 0x00,  // Measurement rate
        0x01, 0x00,  // One navigation solution per measurement
        0x01, 0x01   // GPS time
    };

    uint16_t sampleRateMs = 1000 / sampleRate;
    payload[0]            = sampleRateMs;
    payload[1]            = sampleRateMs >> 8;

    UBXFrame frame{UBXMessage::UBX_CFG_RATE, payload, sizeof(payload)};

    return safeWriteUBXFrame(frame);
}

bool UBXGPSSerial::setPVTMessageRate()
{
    uint8_t payload[] = {
        0x01, 0x07,  // PVT message
        0x01         // Rate = 1 navigation solution update
    };

    UBXFrame frame{UBXMessage::UBX_CFG_MSG, payload, sizeof(payload)};

    return safeWriteUBXFrame(frame);
}

bool UBXGPSSerial::readUBXFrame(UBXFrame& frame)
{
    // Search UBX frame preamble byte by byte
    size_t i = 0;
    while (i < 2)
    {
        uint8_t c;
        if (read(gpsFile, &c, 1) <= 0)  // No more data available
            return false;

        if (c == UBXFrame::PREAMBLE[i])
        {
            frame.preamble[i++] = c;
        }
        else if (c == UBXFrame::PREAMBLE[0])
        {
            i                   = 0;
            frame.preamble[i++] = c;
        }
        else
        {
            i = 0;
            LOG_DEBUG(logger, "Received unexpected byte: {:02x} {:#c}", c, c);
        }
    }

    if (read(gpsFile, &frame.message, 2) <= 0 ||
        read(gpsFile, &frame.payloadLength, 2) <= 0 ||
        read(gpsFile, frame.payload, frame.getRealPayloadLength()) <= 0 ||
        read(gpsFile, frame.checksum, 2) <= 0)
        return false;

    if (!frame.isValid())
    {
        LOG_ERR(logger, "Received invalid UBX frame");
        return false;
    }

    return true;
}

bool UBXGPSSerial::writeUBXFrame(const UBXFrame& frame)
{
    if (!frame.isValid())
    {
        LOG_ERR(logger, "Trying to send an invalid UBX frame");
        return false;
    }

    uint8_t packedFrame[frame.getLength()];
    frame.writePacked(packedFrame);

    if (write(gpsFile, packedFrame, frame.getLength()) < 0)
    {
        LOG_ERR(logger, "Failed to write ubx message");
        return false;
    }

    return true;
}

bool UBXGPSSerial::safeWriteUBXFrame(const UBXFrame& frame)
{
    for (unsigned int i = 0; i < MAX_TRIES; i++)
    {
        if (i > 0)
            LOG_DEBUG(logger, "Retrying (attempt {:#d} of {:#d})...", i + 1,
                      MAX_TRIES);

        if (!writeUBXFrame(frame))
            return false;

        UBXAckFrame ack;

        if (!readUBXFrame(ack))
            continue;

        if (ack.isNack())
        {
            if (ack.getAckMessage() == frame.getMessage())
                LOG_DEBUG(logger, "Received NAK");
            else
                LOG_DEBUG(logger, "Received NAK for different UBX frame {:04x}",
                          static_cast<uint16_t>(ack.getPayload().ackMessage));
            continue;
        }

        if (ack.isAck() && ack.getAckMessage() != frame.getMessage())
        {
            LOG_DEBUG(logger, "Received ACK for different UBX frame {:04x}",
                      static_cast<uint16_t>(ack.getPayload().ackMessage));
            continue;
        }

        return true;
    }

    LOG_ERR(logger, "Gave up after {:#d} tries", MAX_TRIES);
    return false;
}

void UBXGPSSerial::run()
{
    while (!shouldStop())
    {
        UBXPvtFrame pvt;

        // Try to read the message
        if (!readUBXFrame(pvt))
        {
            LOG_DEBUG(logger, "Unable to read a UBX message");
            continue;
        }

        UBXPvtFrame::Payload& pvtP = pvt.getPayload();

        threadSample.gpsTimestamp =
            TimestampTimer::getInstance().getTimestamp();
        threadSample.latitude      = (float)pvtP.lat / 1e7;
        threadSample.longitude     = (float)pvtP.lon / 1e7;
        threadSample.height        = (float)pvtP.height / 1e3;
        threadSample.velocityNorth = (float)pvtP.velN / 1e3;
        threadSample.velocityEast  = (float)pvtP.velE / 1e3;
        threadSample.velocityDown  = (float)pvtP.velD / 1e3;
        threadSample.speed         = (float)pvtP.gSpeed / 1e3;
        threadSample.track         = (float)pvtP.headMot / 1e5;
        threadSample.positionDOP   = (float)pvtP.pDOP / 1e2;
        threadSample.satellites    = pvtP.numSV;
        threadSample.fix           = pvtP.fixType;

        StackLogger::getInstance().updateStack(THID_GPS);
    }
}

}  // namespace Boardcore
