/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Tommaso Lamon
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
#include "SCH16TK10.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{
using SCH16TK10Frames::SPIFrame32;
using SCH16TK10Frames::SPIFrame48;

template <typename Frame>
typename Frame::raw_t SCH16TK10<Frame>::transferFrame(
    typename Frame::raw_t frame)
{
    constexpr int BUFFER_SIZE = Frame::bits / 8;
    uint8_t buf[BUFFER_SIZE];

    for (int i = 0; i < BUFFER_SIZE; i++)
        buf[i] = static_cast<uint8_t>(frame >> (8 * (BUFFER_SIZE - 1 - i)));

    {
        SPITransaction spiTransaction{spiSlave};
        spiTransaction.transfer(buf, sizeof(buf));
    }

    typename Frame::raw_t data = 0;
    for (int i = 0; i < BUFFER_SIZE; i++)
        data = (data << 8) | buf[i];

    return data;
}

template <typename Frame>
void SCH16TK10<Frame>::writeRegister(SCH16TK10Defs::Register target,
                                     uint32_t data)
{
    auto frame = Frame::encode(target, SCH16TK10Defs::Mode::WRITE, data);
    {
        SPITransaction spiTransaction{spiSlave};

        if constexpr (Frame::bits == 32)
            spiTransaction.write32(frame);
        else if constexpr (Frame::bits == 48)
            spiTransaction.write48(frame);
    }
}

template <typename Frame>
typename Frame::Response SCH16TK10<Frame>::readRegister(
    SCH16TK10Defs::Register target)
{
    typename Frame::raw_t encodedData =
        Frame::encode(target, SCH16TK10Defs::Mode::READ, 0);

    transferFrame(encodedData);
    auto rawData = transferFrame(encodedData);

    typename Frame::Response resp = Frame::decode(rawData);
    return resp;
}

template <typename Frame>
bool SCH16TK10<Frame>::checkWhoAmI()
{
    auto resp = readRegister(SCH16TK10Defs::Register::COMP_ID);
    return resp.checkCRC && resp.data == SCH16TK10Defs::WHOAMI;
}

template <typename Frame>
bool SCH16TK10<Frame>::testWrite()
{
    writeRegister(SCH16TK10Defs::Register::SYS_TEST, 1);

    auto resp = readRegister(SCH16TK10Defs::Register::SYS_TEST);
    if (resp.data != 1)
        return false;

    writeRegister(SCH16TK10Defs::Register::SYS_TEST, 0);

    resp = readRegister(SCH16TK10Defs::Register::SYS_TEST);
    if (resp.data != 0)
        return false;

    return true;
}

// Add logger error messages
template <typename Frame>
bool SCH16TK10<Frame>::checkStatus()
{
    auto resp = readRegister(SCH16TK10Defs::Register::STAT_SUM);
    if (resp.data != 0xFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_SUM_SAT);
    if (resp.data != 0xFFFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_COM);
    if (resp.data != 0xFFFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_RATE_COM);
    if (resp.data != 0xFFFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_RATE_X);
    if (resp.data != 0xFFFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_RATE_Y);
    if (resp.data != 0xFFFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_RATE_Z);
    if (resp.data != 0xFFFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_ACC_X);
    if (resp.data != 0xFFFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_ACC_Y);
    if (resp.data != 0xFFFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_ACC_Z);
    if (resp.data != 0xFFFF)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_SYNC_ACTIVE);
    if (resp.data != 0)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::STAT_INFO);
    if (resp.data != 0)
        return false;

    return true;
}

template <typename Frame>
bool SCH16TK10<Frame>::checkConfig()
{
    // Only checking written values -> Might want to expand also to all
    // registers
    auto resp = readRegister(SCH16TK10Defs::Register::CTRL_FILT_RATE);
    if (resp.data != 0x49)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::CTRL_FILT_ACC12);
    if (resp.data != 0x49)
        return false;

    resp = readRegister(SCH16TK10Defs::Register::CTRL_FILT_ACC3);
    if (resp.data != 0x49)
        return false;

    return true;
}

template <typename Frame>
bool SCH16TK10<Frame>::selfTest()
{
    isInit = false;

    if (!testWrite() || !checkWhoAmI() || !checkStatus())
        return false;

    return init();
}

template <typename Frame>
bool SCH16TK10<Frame>::init()
{
    // Set up control registers

    // Filters (200Hz Sampling)
    writeRegister(SCH16TK10Defs::Register::CTRL_FILT_RATE,
                  static_cast<uint32_t>(0x49));
    writeRegister(SCH16TK10Defs::Register::CTRL_FILT_ACC12,
                  static_cast<uint32_t>(0x49));
    writeRegister(SCH16TK10Defs::Register::CTRL_FILT_ACC3,
                  static_cast<uint32_t>(0x49));

    // Enable sensor
    writeRegister(SCH16TK10Defs::Register::CTRL_MODE, 1);

    miosix::Thread::sleep(215);

    // Read once all status registers
    readRegister(SCH16TK10Defs::Register::STAT_SUM);
    readRegister(SCH16TK10Defs::Register::STAT_SUM_SAT);
    readRegister(SCH16TK10Defs::Register::STAT_COM);
    readRegister(SCH16TK10Defs::Register::STAT_RATE_COM);
    readRegister(SCH16TK10Defs::Register::STAT_RATE_X);
    readRegister(SCH16TK10Defs::Register::STAT_RATE_Y);
    readRegister(SCH16TK10Defs::Register::STAT_RATE_Z);
    readRegister(SCH16TK10Defs::Register::STAT_ACC_X);
    readRegister(SCH16TK10Defs::Register::STAT_ACC_Y);
    readRegister(SCH16TK10Defs::Register::STAT_ACC_Z);
    readRegister(SCH16TK10Defs::Register::STAT_SYNC_ACTIVE);
    readRegister(SCH16TK10Defs::Register::STAT_INFO);

    // End of initialization
    writeRegister(SCH16TK10Defs::Register::CTRL_MODE, 3);

    if (checkStatus() && checkStatus() && checkConfig())
        isInit = true;  // Datasheet imposes reading and checking twice all
                        // status registers

    return isInit;
}

// Might want to check crc and error flags in the packet
template <typename Frame>
SCH16TK10Data SCH16TK10<Frame>::sampleImpl()
{
    SCH16TK10Data data            = {};
    bool valid                    = true;

    auto read = [&](SCH16TK10Defs::Register target, auto convert,
                    float sensitivity) -> float
    {
        auto resp = readRegister(target);
        if (!resp.checkCRC ||
            resp.sensorStatus != SCH16TK10Defs::SensorStatus::NORMAL)
            valid = false;

        return convert(resp.data) / sensitivity;
    };

    data.accelerationX = read(SCH16TK10Defs::Register::ACC_X1, Frame::toFloat,
                              Frame::ACC_SENSITIVITY);
    data.accelerationY = read(SCH16TK10Defs::Register::ACC_Y1, Frame::toFloat,
                              Frame::ACC_SENSITIVITY);
    data.accelerationZ = read(SCH16TK10Defs::Register::ACC_Z1, Frame::toFloat,
                              Frame::ACC_SENSITIVITY);

    data.angularSpeedX = read(SCH16TK10Defs::Register::RATE_X1, Frame::toFloatGyro,
                              Frame::RATE_SENSITIVITY);
    data.angularSpeedY = read(SCH16TK10Defs::Register::RATE_Y1, Frame::toFloatGyro,
                              Frame::RATE_SENSITIVITY);
    data.angularSpeedZ = read(SCH16TK10Defs::Register::RATE_Z1, Frame::toFloatGyro,
                              Frame::RATE_SENSITIVITY);

    data.temperature = read(SCH16TK10Defs::Register::TEMP, Frame::toFloatTemp,
                            Frame::TEMP_SENSITIVITY);

    uint64_t ts                = TimestampTimer::getTimestamp();
    data.accelerationTimestamp = ts;
    data.angularSpeedTimestamp = ts;
    data.temperatureTimestamp  = ts;

    if (!valid)
        return lastSample;

    return data;
}

template class SCH16TK10<SPIFrame32>;
template class SCH16TK10<SPIFrame48>;
}  // namespace Boardcore
