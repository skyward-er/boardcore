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

#include "UBXGPS.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

UBXGPS::UBXGPS(SPIBusInterface& spiBus, miosix::GpioPin spiCs,
               SPIBusConfig spiConfig, uint8_t rate)
    : spiSlave(spiBus, spiCs, spiConfig), rate(rate)
{
}

SPIBusConfig UBXGPS::getDefaultSPIConfig()
{
    return SPIBusConfig{SPI::ClockDivider::DIV_256, SPI::Mode::MODE_0};
}

uint8_t UBXGPS::getRate() { return rate; }

bool UBXGPS::init()
{
    if (!reset())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not reset the device");
        return false;
    }

    if (!disableNMEAMessages())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not disable NMEA messages");
        return false;
    }

    if (!setDynamicModelToAirborne4g())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the dynamic model");
        return false;
    }

    if (!setUpdateRate())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the update rate");
        return false;
    }

    if (!setPVTMessageRate())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Could not set the PVT message rate");
        return false;
    }

    return true;
}

bool UBXGPS::selfTest() { return true; }

UBXGPSData UBXGPS::sampleImpl()
{
    UBXPvtFrame pvt;

    if (!readUBXFrame(pvt))
    {
        LOG_ERR(logger, "Have not received a NAV-PVT frame");
        return lastSample;
    }

    UBXPvtFrame::Payload& pvtP = pvt.getPayload();

    UBXGPSData sample;
    sample.gpsTimestamp  = TimestampTimer::getInstance().getTimestamp();
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

bool UBXGPS::reset()
{
    uint8_t payload[] = {
        0x00, 0x00,  // Hoy start
        0x00,        // Hardware reset
        0x00         // Reserved
    };

    UBXFrame frame{UBXMessage::UBX_CFG_RST, payload, sizeof(payload)};

    // The reset message will not be followed by an ack
    if (!writeUBXFrame(frame))
        return false;

    miosix::Thread::sleep(150);

    return true;
}

bool UBXGPS::disableNMEAMessages()
{
    uint8_t payload[] = {
        0x04,                    // SPI port
        0x00,                    // reserved1
        0x00, 0x00,              // txReady
        0x00, 0x32, 0x00, 0x00,  // mode = 0, ffCnt = 50
        0x00, 0x00, 0x00, 0x00,  // reserved2
        0x01, 0x00,              // inProtoMask = UBX
        0x01, 0x00,              // outProtoMask = UBX
        0x00, 0x00,              // flags
        0x00, 0x00               // reserved3
    };

    UBXFrame frame{UBXMessage::UBX_CFG_PRT, payload, sizeof(payload)};

    return safeWriteUBXFrame(frame);
}

bool UBXGPS::setDynamicModelToAirborne4g()
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

bool UBXGPS::setUpdateRate()
{
    uint8_t payload[] = {
        0x00, 0x00,  // Measurement rate
        0x01, 0x00,  // One navigation solution per measurement
        0x01, 0x01   // GPS time
    };

    uint16_t rateMs = 1000 / rate;
    payload[0]      = rateMs;
    payload[1]      = rateMs >> 8;

    UBXFrame frame{UBXMessage::UBX_CFG_RATE, payload, sizeof(payload)};

    return safeWriteUBXFrame(frame);
}

bool UBXGPS::setPVTMessageRate()
{
    uint8_t payload[] = {
        0x01, 0x07,  // PVT message
        0x01         // Rate = 1 navigation solution update
    };

    UBXFrame frame{UBXMessage::UBX_CFG_MSG, payload, sizeof(payload)};

    return safeWriteUBXFrame(frame);
}

bool UBXGPS::readUBXFrame(UBXFrame& frame)
{
    {
        SPITransaction spi{spiSlave};

        // Wait for UBX frame preamble
        bool synchronized = false;
        while (!synchronized)
        {
            synchronized = true;
            for (size_t i = 0; synchronized && i < 2; i++)
            {
                if ((frame.preamble[i] = spi.read()) != UBX_PREAMBLE[i])
                {
                    synchronized = false;
                    if (frame.preamble[i] != UBX_WAIT)
                        LOG_DEBUG(logger, "Received unexpected byte {:#02x}",
                                  frame.preamble[i]);
                }
            }
        }
        spi.read(&frame.message, 2);
        spi.read(&frame.payloadLength, 2);
        spi.read(frame.payload, frame.getRealPayloadLength());
        spi.read(frame.checksum, 2);
    }

    return frame.isValid();
}

bool UBXGPS::writeUBXFrame(const UBXFrame& frame)
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
    }

    return true;
}

bool UBXGPS::safeWriteUBXFrame(const UBXFrame& frame)
{
    if (!writeUBXFrame(frame))
        return false;

    UBXAckFrame ack;

    // Read a frame with the correct length for an ack frame
    if (!readUBXFrame(ack))
    {
        LOG_ERR(logger, "The received UBX frame is not valid");
        return false;
    }

    if (ack.getAckMessage() != frame.getMessage())
    {
        LOG_DEBUG(logger, "Received ACK for a different frame {:#04x}",
                  static_cast<uint16_t>(ack.getPayload().ackMessage));
        return false;
    }

    return ack.isAck();
}

bool UBXGPS::pollReadUBXFrame(UBXMessage message, UBXFrame& response)
{
    UBXFrame request{message, nullptr, 0};

    if (!writeUBXFrame(request))
        return false;

    if (!readUBXFrame(response))
    {
        LOG_ERR(logger, "Invalid UBX frame");
        return false;
    }

    if (response.getMessage() != message)
    {
        LOG_ERR(logger, "UBX frame not recognized");
        return false;
    }

    return true;
}

}  // namespace Boardcore
