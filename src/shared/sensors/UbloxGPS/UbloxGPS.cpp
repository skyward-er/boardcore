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

#include "UbloxGPS.h"

#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

UbloxGPS::UbloxGPS(SPIBusInterface& spiBus, miosix::GpioPin spiCs,
                   SPIBusConfig spiConfig)
    : spiSlave(spiBus, spiCs, spiConfig)
{
}

SPIBusConfig UbloxGPS::getDefaultSPIConfig()
{
    return SPIBusConfig{SPI::ClockDivider::DIV_256, SPI::Mode::MODE_0};
}

bool UbloxGPS::init()
{
    if (!reset() || !setConfiguration())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Initialization failed");
        return false;
    }
    return true;
}

bool UbloxGPS::selfTest() { return true; }

UbloxGPSData UbloxGPS::sampleImpl()
{
    UBXUnpackedFrame res;

    if (!pollReadUBXFrame(UBX_NAV, UBX_NAV_PVT, res))
    {
        LOG_ERR(logger, "Unable to poll UBX-NAV-PVT frame");
        return lastSample;
    }

    UBXPayloadNAVPVT& pvt = (UBXPayloadNAVPVT&)res.payload;

    UbloxGPSData sample;

    sample.gpsTimestamp  = TimestampTimer::getInstance().getTimestamp();
    sample.latitude      = (float)pvt.lat / 1e7;
    sample.longitude     = (float)pvt.lon / 1e7;
    sample.height        = (float)pvt.height / 1e3;
    sample.velocityNorth = (float)pvt.velN / 1e3;
    sample.velocityEast  = (float)pvt.velE / 1e3;
    sample.velocityDown  = (float)pvt.velD / 1e3;
    sample.speed         = (float)pvt.gSpeed / 1e3;
    sample.track         = (float)pvt.headMot / 1e5;
    sample.satellites    = pvt.numSV;
    sample.fix           = pvt.fixType != 0;  // All types of fix are accepted

    return sample;
}

bool UbloxGPS::reset()
{
    uint8_t payload[] = {
        0x00, 0x00,  // navBbrMask (Hot start)
        0x00,        // Hardware reset immediately
        0x00         // Reserved
    };

    UBXUnpackedFrame frame{UBX_CFG, UBX_CFG_RST, payload,
                           std::extent<decltype(payload)>::value};

    if (!writeUBXFrame(frame))
    {
        return false;
    }

    miosix::Thread::sleep(100);

    return true;
}

bool UbloxGPS::setConfiguration()
{
    uint8_t payload[] = {
        0x00,                    // Version
        0x07,                    // All layers
        0x00, 0x00,              // Reserved
        0x21, 0x00, 0x11, 0x20,  // CFG-NAVSPG-DYNMODEL
        0x08,                    // = AIR4
        0x03, 0x00, 0x51, 0x10,  // CFG-I2C-ENABLED
        0x00, 0x00,              // = false
        0x06, 0x00, 0x64, 0x10,  // CFG-SPI-ENABLED
        0x00, 0x01,              // = true
        0x01, 0x00, 0x79, 0x10,  // CFG-SPIINPROT-UBX
        0x00, 0x01,              // = true
        0x02, 0x00, 0x79, 0x10,  // CFG-SPIINPROT-NMEA
        0x00, 0x00,              // = false
        0x03, 0x00, 0x79, 0x10,  // CFG-SPIINPROT-RTCM3X
        0x00, 0x00,              // = false
        0x01, 0x00, 0x7a, 0x10,  // CFG-SPIOUTPROT-UBX
        0x00, 0x01,              // = true
        0x02, 0x00, 0x7a, 0x10,  // CFG-SPIOUTPROT-NMEA
        0x00, 0x00,              // = false
        0x05, 0x00, 0x52, 0x10,  // CFG-UART1-ENABLED
        0x00, 0x00,              // = false
        0x05, 0x00, 0x53, 0x10,  // CFG-UART2-ENABLED
        0x00, 0x00,              // = false
        0x01, 0x00, 0x65, 0x10,  // CFG-USB-ENABLED
        0x00, 0x00               // = false
    };

    UBXUnpackedFrame frame{UBX_CFG, UBX_CFG_VALSET, payload,
                           std::extent<decltype(payload)>::value};

    return safeWriteUBXFrame(frame);
}

bool UbloxGPS::safeWriteUBXFrame(const UBXUnpackedFrame& frame)
{
    while (true)
    {
        if (!writeUBXFrame(frame))
        {
            return false;
        }

        UBXUnpackedFrame res;

        if (!readUBXFrame(res))
        {
            return false;
        }

        if (res.cls == UBX_ACK && res.id == UBX_ACK_ACK)
        {
            UBXPayloadACK& ack = (UBXPayloadACK&)res.payload;

            if (ack.clsID != frame.cls || ack.msgID != frame.id)
            {
                break;
            }

            LOG_DEBUG(logger,
                      "Received ACK for frame (class: {:#02x}, id: {:#02x})",
                      res.cls, res.id);

            return true;
        }
        else if (res.cls == UBX_ACK && res.id == UBX_ACK_NAK)
        {
            UBXPayloadACK& nak = (UBXPayloadACK&)res.payload;

            if (nak.clsID != frame.cls || nak.msgID != frame.id)
            {
                break;
            }

            LOG_DEBUG(logger,
                      "Received NAK for frame (class: {:#02x}, id: {:#02x})",
                      res.cls, res.id);
        }
        else
        {
            LOG_ERR(logger,
                    "UBX frame not recognized (class: {:#02x}, id: {:#02x})",
                    res.cls, res.id);

            break;
        }
    }

    return false;
}

bool UbloxGPS::pollReadUBXFrame(uint8_t cls, uint8_t id,
                                UBXUnpackedFrame& frame)
{
    UBXUnpackedFrame req{cls, id, nullptr, 0};

    if (!writeUBXFrame(req))
    {
        return false;
    }

    UBXUnpackedFrame res;

    if (!readUBXFrame(res))
    {
        return false;
    }

    if (res.cls != cls || res.id != id)
    {
        LOG_ERR(logger,
                "UBX frame not recognized (class: {:#02x}, id: {:#02x})",
                res.cls, res.id);
        return false;
    }

    return true;
}

bool UbloxGPS::writeUBXFrame(const UBXUnpackedFrame& frame)
{
    if (!frame.isValid())
    {
        LOG_ERR(logger, "UBX frame to write is invalid");
        return false;
    }

    SPITransaction spi{spiSlave};
    uint8_t packedFrame[UBX_MAX_FRAME_LENGTH];
    frame.writePacked(packedFrame);
    spi.write(packedFrame, frame.getLength());

    return true;
}

bool UbloxGPS::readUBXFrame(UBXUnpackedFrame& frame)
{
    SPITransaction spi{spiSlave};
    uint8_t packedFrame[UBX_MAX_FRAME_LENGTH];
    spi.read(packedFrame, UBX_MAX_FRAME_LENGTH);
    frame.readPacked(packedFrame);

    if (!frame.isValid())
    {
        LOG_ERR(logger, "UBX frame received is invalid");
        return false;
    }

    return true;
}

}  // namespace Boardcore
