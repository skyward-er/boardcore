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
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>

using namespace miosix;

namespace Boardcore
{

UBXGPSSerial::UBXGPSSerial(USARTInterface::Baudrate baudrate,
                           uint8_t sampleRate, USARTType* usartNumber,
                           USARTInterface::Baudrate defaultBaudrate)
{
    this->usart           = nullptr;
    this->baudrate        = baudrate;
    this->defaultBaudrate = defaultBaudrate;
    this->sampleRate      = sampleRate;
    this->usartNumber     = usartNumber;
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

    miosix::Thread::sleep(100);

    LOG_DEBUG(logger, "Resetting the device...");

    // if (!reset())
    // {
    //     lastError = SensorErrors::INIT_FAIL;
    //     LOG_ERR(logger, "Could not reset the device");
    //     return false;
    // }

    LOG_DEBUG(logger, "Setting the UBX protocol...");

    if (!setBaudrateAndUBX(false))
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the UBX protocol");
        return false;
    }

    miosix::Thread::sleep(100);

    LOG_DEBUG(logger, "Setting the dynamic model...");

    if (!setDynamicModelToAirborne4g())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the dynamic model");
        return false;
    }

    miosix::Thread::sleep(100);

    LOG_DEBUG(logger, "Setting the sample rate...");

    if (!setSampleRate())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the sample rate");
        return false;
    }

    miosix::Thread::sleep(100);

    LOG_DEBUG(logger, "Setting the PVT message rate...");

    if (!setPVTMessageRate())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the PVT message rate");
        return false;
    }

    miosix::Thread::sleep(100);

    this->start();

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

bool UBXGPSSerial::setBaudrateAndUBX(bool safe)
{
    uint8_t payload[] = {
        0x01,                    // UART port
        0x00,                    // Reserved
        0x00, 0x00,              // txReady
        0xc0, 0x08, 0x00, 0x00,  // 8bit, no parity, 1 stop bit
        0x00, 0x00, 0x00, 0x00,  // Baudrate
        0x01, 0x00,              // inProtoMask = UBX
        0x01, 0x00,              // outProtoMask = UBX
        0x00, 0x00,              // flags
        0x00, 0x00               // reserved2
    };

    int baud = (int)baudrate;

    // Prepare baudrate
    payload[8]  = baud;
    payload[9]  = baud >> 8;
    payload[10] = baud >> 16;
    payload[11] = baud >> 24;

    UBXFrame frame{UBXMessage::UBX_CFG_PRT, payload, sizeof(payload)};

    if (safe)
        return safeWriteUBXFrame(frame);
    else
        return writeUBXFrame(frame);
}

bool UBXGPSSerial::setSerialCommunication()
{
    usart = new USART(usartNumber, defaultBaudrate);
    usart->init();
    // Change the baudrate only if it is different than the default
    if (baudrate != defaultBaudrate)
    {
        miosix::Thread::sleep(100);
        // Change baudrate
        if (!setBaudrateAndUBX(false))
        {
            return false;
        };
    }

    miosix::Thread::sleep(100);
    usart->setBaudrate(baudrate);

    return true;
}

bool UBXGPSSerial::setDynamicModelToAirborne4g()
{
    uint8_t payload[] = {
        0x01, 0x00,  // Parameters bitmask, apply dynamic model configuration
        0x08,        // Dynamic model = airborne 4g
        0x00,        // Fix mode
        0x00, 0x00, 0x00, 0x00,  // Fixed altitude for 2D mode
        0x00, 0x00, 0x00, 0x00,  // Fixed altitude variance for 2D mode
        0x00,        // Minimum elevation for a GNSS satellite to be used
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
        if (usart->read(&c, 1) <= 0)  // No more data available
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
            // LOG_DEBUG(logger, "Received unexpected byte: {:02x} {:#c}", c,
            // c);
        }
    }

    if (usart->read(&frame.message, 2) <= 0 ||
        usart->read(&frame.payloadLength, 2) <= 0 ||
        usart->read(frame.payload, frame.getRealPayloadLength()) <= 0 ||
        usart->read(frame.checksum, 2) <= 0)
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

    if (usart->write(packedFrame, frame.getLength()) < 0)
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

        // Lock the mutex
        Lock<FastMutex> l(mutex);
        threadSample.gpsTimestamp  = TimestampTimer::getTimestamp();
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
        threadSample.ubxTime       = {.year       = pvtP.year,
                                      .month      = pvtP.month,
                                      .day        = pvtP.day,
                                      .hour       = pvtP.hour,
                                      .minute     = pvtP.min,
                                      .second     = pvtP.sec,
                                      .nanosecond = pvtP.nano,
                                      .accuracy   = pvtP.tAcc};

        StackLogger::getInstance().updateStack(THID_GPS);
    }
}

}  // namespace Boardcore
