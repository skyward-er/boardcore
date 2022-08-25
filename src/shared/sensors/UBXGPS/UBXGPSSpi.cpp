/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Davide Bonomini, Davide Mor, Alberto Nidasio, Damiano Amatruda
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

#include "UBXGPSSpi.h"

#include <drivers/timer/TimestampTimer.h>
#include <interfaces/endianness.h>

using namespace miosix;

namespace Boardcore
{

constexpr uint16_t UBXFrame::MAX_PAYLOAD_LENGTH;
constexpr uint16_t UBXFrame::MAX_FRAME_LENGTH;
constexpr uint8_t UBXFrame::PREAMBLE[];
constexpr uint8_t UBXFrame::WAIT;

constexpr float UBXGPSSpi::MS_TO_TICK;

constexpr unsigned int UBXGPSSpi::RESET_SLEEP_TIME;
constexpr unsigned int UBXGPSSpi::READ_TIMEOUT;
constexpr unsigned int UBXGPSSpi::MAX_TRIES;

UBXGPSSpi::UBXGPSSpi(SPIBusInterface& spiBus, miosix::GpioPin spiCs,
                     SPIBusConfig spiConfig, uint8_t sampleRate)
    : spiSlave(spiBus, spiCs, spiConfig), sampleRate(sampleRate)
{
}

SPIBusConfig UBXGPSSpi::getDefaultSPIConfig()
{
    return SPIBusConfig{SPI::ClockDivider::DIV_16, SPI::Mode::MODE_0,
                        SPI::BitOrder::MSB_FIRST, 10, 10};
}

uint8_t UBXGPSSpi::getSampleRate() { return sampleRate; }

bool UBXGPSSpi::init()
{
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

bool UBXGPSSpi::selfTest() { return true; }

UBXGPSData UBXGPSSpi::sampleImpl()
{
    UBXPvtFrame pvt;

    if (!readUBXFrame(pvt))
        return lastSample;

    UBXPvtFrame::Payload& pvtP = pvt.getPayload();

    UBXGPSData sample;
    sample.gpsTimestamp  = TimestampTimer::getTimestamp();
    sample.latitude      = (float)pvtP.lat / 1e7;
    sample.longitude     = (float)pvtP.lon / 1e7;
    sample.height        = (float)pvtP.height / 1e3;
    sample.velocityNorth = (float)pvtP.velN / 1e3;
    sample.velocityEast  = (float)pvtP.velE / 1e3;
    sample.velocityDown  = (float)pvtP.velD / 1e3;
    sample.speed         = (float)pvtP.gSpeed / 1e3;
    sample.track         = (float)pvtP.headMot / 1e5;
    sample.positionDOP   = (float)pvtP.pDOP / 1e2;
    sample.satellites    = pvtP.numSV;
    sample.fix           = pvtP.fixType;

    return sample;
}

bool UBXGPSSpi::reset()
{
    uint8_t payload[] = {
        0x00, 0x00,  // Hot start
        0x01,        // Controlled software reset
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

bool UBXGPSSpi::setUBXProtocol()
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

bool UBXGPSSpi::setDynamicModelToAirborne4g()
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

bool UBXGPSSpi::setSampleRate()
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

bool UBXGPSSpi::setPVTMessageRate()
{
    uint8_t payload[] = {
        0x01, 0x07,  // PVT message
        0x01         // Rate = 1 navigation solution update
    };

    UBXFrame frame{UBXMessage::UBX_CFG_MSG, payload, sizeof(payload)};

    return safeWriteUBXFrame(frame);
}

bool UBXGPSSpi::readUBXFrame(UBXFrame& frame)
{
    long long start = miosix::getTick();
    long long end   = start + READ_TIMEOUT * MS_TO_TICK;

    {
        spiSlave.bus.select(spiSlave.cs);

        // Search UBX frame preamble byte by byte
        size_t i     = 0;
        bool waiting = false;
        while (i < 2)
        {
            if (miosix::getTick() >= end)
            {
                LOG_ERR(logger, "Timeout for read expired");
                spiSlave.bus.deselect(spiSlave.cs);
                Thread::sleep(1);  // GPS minimum time after deselect
                return false;
            }

            uint8_t c = spiSlave.bus.read();

            if (c == UBXFrame::PREAMBLE[i])
            {
                waiting             = false;
                frame.preamble[i++] = c;
            }
            else if (c == UBXFrame::PREAMBLE[0])
            {
                i                   = 0;
                waiting             = false;
                frame.preamble[i++] = c;
            }
            else if (c == UBXFrame::WAIT)
            {
                i = 0;
                if (!waiting)
                {
                    waiting = true;
                    // LOG_DEBUG(logger, "Device is waiting...");
                }
            }
            else
            {
                i       = 0;
                waiting = false;
                LOG_DEBUG(logger, "Received unexpected byte: {:02x} {:#c}", c,
                          c);
            }
        }

        frame.message       = swapBytes16(spiSlave.bus.read16());
        frame.payloadLength = swapBytes16(spiSlave.bus.read16());
        spiSlave.bus.read(frame.payload, frame.getRealPayloadLength());
        spiSlave.bus.read(frame.checksum, 2);

        spiSlave.bus.deselect(spiSlave.cs);
        Thread::sleep(1);  // GPS minimum time after deselect
    }

    if (!frame.isValid())
    {
        LOG_ERR(logger, "Received invalid UBX frame");
        return false;
    }

    return true;
}

bool UBXGPSSpi::writeUBXFrame(const UBXFrame& frame)
{
    if (!frame.isValid())
    {
        LOG_ERR(logger, "Trying to send an invalid UBX frame");
        return false;
    }

    uint8_t packedFrame[frame.getLength()];
    frame.writePacked(packedFrame);

    {
        SPITransaction spi{spiSlave};
        spi.write(packedFrame, frame.getLength());
        Thread::sleep(1);  // GPS minimum time after deselect
    }

    return true;
}

bool UBXGPSSpi::safeWriteUBXFrame(const UBXFrame& frame)
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

}  // namespace Boardcore
