/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Aldas Lenkšas
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

#include "AS5047U.h"

#include <units/Angle.h>

#include "AS5047UFrames.h"

namespace Boardcore
{

using namespace AS5047UDefs;
using namespace AS5047UFrames;

template <Frame ReadFrame>
ReadFrame::raw_t AS5047U<ReadFrame>::transferFrame(ReadFrame::raw_t frame)
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

// For writing to registers, 24-bit frames are always used.
template <Frame ReadFrame>
void AS5047U<ReadFrame>::writeRegister(Registers target, uint16_t data)
{
    auto frame = SPIFrame24::encode(target, Mode::WRITE, data);
    {
        SPITransaction spiTransaction{spiSlave};
        spiTransaction.write24(frame);
    }
}

template <Frame ReadFrame>
ReadFrame::Response AS5047U<ReadFrame>::readRegister(Registers target,
                                                     const std::string& regName,
                                                     SensorErrors errorOnFail)
{
    ReadFrame::raw_t encodedData =
        ReadFrame::encode(target, SCH16TK10Defs::Mode::READ, 0);

    transferFrame(encodedData);
    auto rawData = transferFrame(encodedData);

    ReadFrame::Response resp = ReadFrame::decode(rawData);

    if (!resp.okCRC)
    {
        LOG_ERR("CRC wrong when reading {}", regName);
        lastError = errorOnFail;
        dumpErrorRegister(false);
    }
    if (resp.error)
    {
        LOG_ERR("Error occurred when reading {}", regName);
        lastError = errorOnFail;
        dumpErrorRegister(true);
    }
    if (resp.warning)
    {
        LOG_WARN("Warning occurred when reading {}", regName);
        dumpErrorRegister(false);
    }

    return resp;
}

template <Frame ReadFrame>
AS5047U<ReadFrame>::AS5047U(SPIBusInterface& spiBus, miosix::GpioPin cs,
                            SPIBusConfig& spiCfg, AS5047UConfig& sensorCfg)
    : spiSlave{spiBus, cs, spiCfg}, initialized{false}, config{sensorCfg}
{
}

template <Frame ReadFrame>
AS5047U<ReadFrame>::AS5047U(SPIBusInterface& spiBus, miosix::GpioPin cs,
                            AS5047UConfig& sensorCfg)
    : spiSlave{spiBus, cs, getDefaultSPIConfig()}, initialized{false},
      config{sensorCfg}
{
}

template <Frame ReadFrame>
SPIBusConfig AS5047U<ReadFrame>::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_128;
    spiConfig.mode         = SPI::Mode::MODE_1;
    spiConfig.bitOrder     = SPI::Order::MSB_FIRST;
    return spiConfig;
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::init()
{
    if (initialized)
    {
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }

    if (!initDisable())
        return false;
    if (!initSettings1())
        return false;
    if (!initSettings2())
        return false;

    initialized = true;
    return true;
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::initDisable()
{
    ReadFrame::Response reading =
        readRegister(Registers::DISABLE, "DISABLE", SensorErrors::INIT_FAIL);
    if (!reading.okCRC || reading.error)
        return false;

    uint16_t disableVal =
        reading.data & 0b0111100;  // na field remains unchanged
    disableVal |= ((static_cast<uint16_t>(config.uvw) & 0b1)) |
                  ((static_cast<uint16_t>(config.abi) & 0b1) << 1) |
                  ((static_cast<uint16_t>(config.filter) & 0b1) << 6);

    return writeAndCheck("DISABLE", Registers::DISABLE, disableVal,
                         SensorErrors::INIT_FAIL);
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::initSettings1()
{
    ReadFrame::Response reading = readRegister(
        Registers::SETTINGS1, "SETTINGS1", SensorErrors::INIT_FAIL);
    if (!reading.okCRC || reading.error)
        return false;

    uint16_t settings1 =
        reading.data & 0b1100000;  // last two bits remain unchanged

    settings1 |= ((static_cast<uint16_t>(config.kMax) & 0b111)) |
                 ((static_cast<uint16_t>(config.kMin) & 0b111) << 3);

    return writeAndCheck("SETTINGS1", Registers::SETTINGS1, settings1,
                         SensorErrors::INIT_FAIL);
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::initSettings2()
{
    // As ABI, UVW, PWM are not supported, the following values are ignored:
    // IWIDTH, DIR, UVW_ABI, ABI_DEC
    uint16_t settings2 =
        ((static_cast<uint16_t>(config.noiseSet) & 0b1) << 1) |
        ((static_cast<uint16_t>(config.daeCompensation) & 0b1) << 4) |
        ((static_cast<uint16_t>(config.dataSource) & 0b1) << 6) |
        ((static_cast<uint16_t>(config.pwm) & 0b1) << 7);

    return writeAndCheck("SETTINGS2", Registers::SETTINGS2, settings2,
                         SensorErrors::INIT_FAIL);
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::writeAndCheck(const std::string& regName,
                                       const Registers reg,
                                       const uint16_t value,
                                       SensorErrors errorOnFail)
{
    writeRegister(reg, value);
    ReadFrame::Response reading = readRegister(reg, regName, errorOnFail);

    if (!reading.okCRC || reading.error)
        return false;

    initialized = reading.data == value;

    if (!initialized)
    {
        LOG_ERR(logger,
                "{} register after init: {:X}, expected "
                "value: {:X}.",
                regName, reading.data, value);
        lastError = errorOnFail;
        dumpErrorRegister(false);
        return false;
    }

    return true;
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::selfTest()
{
    return true;
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::resetAngleZero()
{
    ReadFrame::Response daecAngInt =
        readRegister(AS5047DDefs::Registers::ANGLECOM, "ANGLECOM",
                     SensorErrors::COMMAND_FAILED);
    if (!daecAngInt.okCRC || daecAngInt.error)
        return false;

    uint16_t angle = daecAngInt.data;

    return writeAndCheck(AS5047DDefs::Registers::ZPOSM, (angle >> 6) & 0xFF,
                         SensorErrors::COMMAND_FAILED) ||
           writeAndCheck(AS5047DDefs::Registers::ZPOSL, angle & 0x3F,
                         SensorErrors::COMMAND_FAILED);
}

template <Frame ReadFrame>
AS5047UData AS5047U<ReadFrame>::sampleImpl()
{
    if (!initialized)
    {
        LOG_ERR(logger, "Sensor not initialized");
        lastError = SensorErrors::NOT_INIT;
        return lastSample;
    }

    uint64_t timestamp = TimestampTimer::getTimestamp();

    // Reading is done from ANGLECOM register, use enableDAEC() or disableDAEC()
    // to change.
    ReadFrame::Response reading = readRegister(Registers::ANGLECOM, "ANGLECOM",
                                               SensorErrors::COMMAND_FAILED);
    if (!reading.okCRC || reading.error)
        return lastSample;

    Units::Angle::Degree daecAngle = Units::Angle::Degree(
        static_cast<float>(reading.data) * AS5047UDefs::SPI_ANGLE_RES);

    // Returns in radians, similar to AS5047D sensor.
    return AS5047UData(timestamp,
                       static_cast<Units::Angle::Radian>(daecAngle).value());
}

template <Frame ReadFrame>
void AS5047U<ReadFrame>::dumpErrorRegister(bool readDiagnostic)
{
    ReadFrame::Response errRegResponse =
        readRegister(Registers::ERRFL, "ERRFL", SensorErrors::COMMAND_FAILED);

    uint16_t errData = errRegResponse.data;

    LOG_ERR(logger,
            "An error has occurred, dumping error register ERRFL.  "
            "CORDIC OVERFLOW: {},"
            "OffCompNotFinished: {},"
            "Watchdog: {},"
            "CRC error: {},"
            "Command error: {},"
            "Framing error: {},"
            "P2RAM error: {},"
            "P2RAM warning: {},"
            "Agc warning: {}.",
            static_cast<bool>((errData >> 10) & 0b1),  // CORDIC Overflow
            static_cast<bool>((errData >> 9) & 0b1),   // OffCompNotFinished
            !static_cast<bool>((errData >> 7) & 0b1),  // WDTST
            static_cast<bool>((errData >> 6) & 0b1),   // CRC error
            static_cast<bool>((errData >> 5) & 0b1),   // Command error
            static_cast<bool>((errData >> 4) & 0b1),   // Framing error
            static_cast<bool>((errData >> 3) & 0b1),   // P2RAM error
            static_cast<bool>((errData >> 2) & 0b1),   // P2RAM warning
            static_cast<bool>((errData >> 1) & 0b1),   // MagHalf warning
            static_cast<bool>(errData & 0b1)           // Agc-warning warning
    );

    if (!readDiagnostic)
        return;

    ReadFrame::Response diaRegResponse =
        readRegister(Registers::DIA, "DIA", SensorErrors::COMMAND_FAILED);

    uint16_t errData = diaRegResponse.data;

    LOG_ERR(logger,
            "Dumping diagnostic register DIA.  "
            "SPI_cnt: {},"
            "AGC finished: {},"
            "Off comp finished: {},"
            "SinOff finished: {},"
            "CosOff finished: {},"
            "MagHalf flag: {},"
            "AGC high: {},"
            "AGC low: {},"
            "CORDIC overflow: {},"
            "loops finished: {},"
            "Vdd mode: {}.",
            static_cast<uint8_t>((errData >> 11) & 0b11),  // SPI_CNT
            static_cast<bool>((errData >> 9) & 0b1),       // AGC_finished
            static_cast<bool>((errData >> 8) & 0b1),       // OFF comp finished
            static_cast<bool>((errData >> 7) & 0b1),       // SinOff finished
            static_cast<bool>((errData >> 6) & 0b1),       // CosOff finished
            static_cast<bool>((errData >> 5) & 0b1),       // MagHalf flag
            static_cast<bool>((errData >> 4) & 0b1),       // Comp_h
            static_cast<bool>((errData >> 3) & 0b1),       // Comp_l
            static_cast<bool>((errData >> 2) & 0b1),       // cordic overflow
            static_cast<bool>(errData & 0b1)               // Vdd mode
    );
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::enableDAEC()
{
    return setDAECStatus(DAECStatus::ON);
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::disableDAEC()
{
    return setDAECStatus(DAECStatus::OFF);
}

template <Frame ReadFrame>
bool AS5047U<ReadFrame>::setDAECStatus(DAECStatus status)
{
    ReadFrame::Result reading = readRegister(Registers::SETTINGS2, "SETTINGS2",
                                             SensorErrors::COMMAND_FAILED);
    if (!reading.okCRC || reading.error)
        return false;

    uint16_t settings2 = reading.data & 0b10111111;  // leave the rest unchanged
    settings2 |= (static_cast<uint16_t>(status) & 0b1 << 6);

    config.dataSource = dataSource;
    lastError         = SensorErrors::NO_ERRORS;

    return writeAndCheck("SETTINGS2", Registers::SETTINGS2, settings2,
                         SensorErrors::COMMAND_FAILED);
}

template class AS5047U<SPIFrame16>;
template class AS5047U<SPIFrame24>;

}  // namespace Boardcore
