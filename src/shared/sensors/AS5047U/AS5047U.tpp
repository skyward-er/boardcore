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

#include <units/Angle.h>

#include "AS5047U.h"
#include "AS5047UFrames.h"

namespace Boardcore
{

template <AS5047UFrames::Frame ReadFrame>
typename ReadFrame::raw_t AS5047U<ReadFrame>::transferFrame(
    typename ReadFrame::raw_t frame)
{
    constexpr int BUFFER_SIZE = ReadFrame::bits / 8;
    uint8_t buf[BUFFER_SIZE];

    for (int i = 0; i < BUFFER_SIZE; i++)
        buf[i] = static_cast<uint8_t>(frame >> (8 * (BUFFER_SIZE - 1 - i)));

    {
        SPITransaction spiTransaction{spiSlave};
        spiTransaction.transfer(buf, sizeof(buf));
    }

    typename ReadFrame::raw_t data = 0;
    for (int i = 0; i < BUFFER_SIZE; i++)
        data = (data << 8) | buf[i];

    return data;
}

// For writing to registers, 24-bit frames are always used.
template <AS5047UFrames::Frame ReadFrame>
void AS5047U<ReadFrame>::writeRegister(AS5047UDefs::Registers target,
                                       uint16_t data)
{
    auto commandFrame = AS5047UFrames::SPIFrame24::encodeWriteCommand(target);
    auto dataFrame    = AS5047UFrames::SPIFrame24::encodeWriteData(data);

    // Reset the sampling state as the next time
    // reading from data register (ANGLECOM) is called, the first
    // sample is not valid.
    isFirstSample = true;

    {
        SPITransaction spiTransaction{spiSlave};
        spiTransaction.transfer24(commandFrame);
    }
    // Temp - might need delay
    {
        SPITransaction SPITransaction{spiSlave};
        SPITransaction.transfer24(dataFrame);
    }
}

template <AS5047UFrames::Frame ReadFrame>
typename ReadFrame::Response AS5047U<ReadFrame>::readRegister(
    AS5047UDefs::Registers target, SensorErrors errorOnFail)
{
    typename ReadFrame::raw_t encodedData =
        ReadFrame::encodeReadCommand(target);

    // Reset the state for sampling since calling this function
    // is only when we ask to read other registers than the data sample.
    isFirstSample = true;

    transferFrame(encodedData);
    // Temp - Might need delay
    auto rawData = transferFrame(encodedData);

    typename ReadFrame::Response resp = ReadFrame::decode(rawData);
    checkResponseErrors(target, resp, errorOnFail);
    return resp;
}

template <AS5047UFrames::Frame ReadFrame>
void AS5047U<ReadFrame>::checkResponseErrors(AS5047UDefs::Registers target,
                                             typename ReadFrame::Response resp,
                                             SensorErrors errorOnFail)
{
    if (!resp.okCRC)
    {
        LOG_ERR(logger, "CRC wrong when reading {}",
                static_cast<uint16_t>(target));
        lastError = errorOnFail;
        dumpErrorRegister(false);
    }
    if (resp.error)
    {
        LOG_ERR(logger, "Error occurred when reading {}",
                static_cast<uint16_t>(target));
        lastError = errorOnFail;
        dumpErrorRegister(true);
    }
    if (resp.warning)
    {
        LOG_WARN(logger, "Warning occurred when reading {}",
                 static_cast<uint16_t>(target));
        dumpErrorRegister(false);
    }
}

template <AS5047UFrames::Frame ReadFrame>
AS5047U<ReadFrame>::AS5047U(SPIBusInterface& spiBus, miosix::GpioPin cs,
                            SPIBusConfig& spiCfg, AS5047UConfig& sensorCfg)
    : spiSlave{spiBus, cs, spiCfg}, initialized{false}, config{sensorCfg}
{
}

template <AS5047UFrames::Frame ReadFrame>
AS5047U<ReadFrame>::AS5047U(SPIBusInterface& spiBus, miosix::GpioPin cs,
                            AS5047UConfig& sensorCfg)
    : spiSlave{spiBus, cs, getDefaultSPIConfig()}, initialized{false},
      config{sensorCfg}
{
}

template <AS5047UFrames::Frame ReadFrame>
SPIBusConfig AS5047U<ReadFrame>::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_128;
    spiConfig.mode         = SPI::Mode::MODE_1;
    spiConfig.bitOrder     = SPI::Order::MSB_FIRST;
    return spiConfig;
}

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::init()
{
    if (initialized)
    {
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }

    isFirstSample = true;

    return initialized = (initDisableRegister() && initSettings1Register() &&
                          initSettings2Register());
}

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::initDisableRegister()
{
    typename ReadFrame::Response reading =
        readRegister(AS5047UDefs::Registers::DISABLE, SensorErrors::INIT_FAIL);
    if (!reading.okCRC || reading.error)
        return false;

    uint16_t disableVal =
        reading.data & 0b0111100;  // na field remains unchanged
    disableVal |= (0b1);           // UVW OFF
    disableVal |= (0b1 << 1);      // ABI OFF
    disableVal |= ((static_cast<uint16_t>(config.filterOn) & 0b1) << 6);

    return writeAndCheck(AS5047UDefs::Registers::DISABLE, disableVal,
                         SensorErrors::INIT_FAIL);
}

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::initSettings1Register()
{
    typename ReadFrame::Response reading = readRegister(
        AS5047UDefs::Registers::SETTINGS1, SensorErrors::INIT_FAIL);
    if (!reading.okCRC || reading.error)
        return false;

    uint16_t settings1 =
        reading.data & 0b1100000;  // last two bits remain unchanged

    settings1 |= ((static_cast<uint16_t>(config.kMax) & 0b111)) |
                 ((static_cast<uint16_t>(config.kMin) & 0b111) << 3);

    return writeAndCheck(AS5047UDefs::Registers::SETTINGS1, settings1,
                         SensorErrors::INIT_FAIL);
}

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::initSettings2Register()
{
    // As ABI, UVW, PWM are not supported, the following values are ignored:
    // IWIDTH, DIR, UVW_ABI, ABI_DEC
    uint16_t settings2 =
        ((static_cast<uint16_t>(config.noiseSet) & 0b1) << 1) |
        ((static_cast<uint16_t>(config.daecOn) & 0b1) << 4) |
        ((static_cast<uint16_t>(config.daecSource) & 0b1) << 6);

    return writeAndCheck(AS5047UDefs::Registers::SETTINGS2, settings2,
                         SensorErrors::INIT_FAIL);
}

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::writeAndCheck(const AS5047UDefs::Registers reg,
                                       const uint16_t value,
                                       SensorErrors errorOnFail)
{
    writeRegister(reg, value);
    typename ReadFrame::Response reading = readRegister(reg, errorOnFail);

    if (!reading.okCRC || reading.error)
        return false;

    initialized = reading.data == value;

    if (!initialized)
    {
        LOG_ERR(logger,
                "{} register after init: {:X}, expected "
                "value: {:X}.",
                static_cast<uint16_t>(reg), reading.data, value);
        lastError = errorOnFail;
        dumpErrorRegister(false);
        return false;
    }

    return true;
}

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::selfTest()
{
    return true;
}

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::resetAngleZero()
{
    typename ReadFrame::Response daecAngInt = readRegister(
        AS5047UDefs::Registers::ANGLECOM, SensorErrors::COMMAND_FAILED);
    if (!daecAngInt.okCRC || daecAngInt.error)
        return false;

    uint16_t angle = daecAngInt.data;

    return writeAndCheck(AS5047UDefs::Registers::ZPOSM, (angle >> 6) & 0xFF,
                         SensorErrors::COMMAND_FAILED) ||
           writeAndCheck(AS5047UDefs::Registers::ZPOSL, angle & 0x3F,
                         SensorErrors::COMMAND_FAILED);
}

template <AS5047UFrames::Frame ReadFrame>
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
    typename ReadFrame::raw_t readCommand =
        ReadFrame::encodeReadCommand(AS5047UDefs::Registers::ANGLECOM);
    typename ReadFrame::raw_t rawData = transferFrame(readCommand);

    typename ReadFrame::Response resp = ReadFrame::decode(rawData);
    checkResponseErrors(AS5047UDefs::Registers::ANGLECOM, resp,
                        SensorErrors::COMMAND_FAILED);

    if (isFirstSample || !resp.okCRC || resp.error)
    {
        // Discard the first sample since the result to a read command
        // is transmitted on the next read command.
        isFirstSample = false;

        return lastSample;
    }

    Units::Angle::Degree daecAngle = Units::Angle::Degree(
        static_cast<float>(resp.data) * AS5047UDefs::SPI_ANGLE_RES);

    // Returns in radians, similar to AS5047D sensor.
    return AS5047UData(timestamp,
                       static_cast<Units::Angle::Radian>(daecAngle).value());
}

template <AS5047UFrames::Frame ReadFrame>
void AS5047U<ReadFrame>::dumpErrorRegister(bool readDiagnostic)
{
    typename ReadFrame::Response errRegResponse = readRegister(
        AS5047UDefs::Registers::ERRFL, SensorErrors::COMMAND_FAILED);

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

    typename ReadFrame::Response diaRegResponse =
        readRegister(AS5047UDefs::Registers::DIA, SensorErrors::COMMAND_FAILED);

    errData = diaRegResponse.data;

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

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::enableDAEC()
{
    return setDAECStatus(AS5047UDefs::DAECompensation::ON,
                         AS5047UDefs::DAECSource::ANGLECOM);
}

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::disableDAEC()
{
    return setDAECStatus(AS5047UDefs::DAECompensation::OFF,
                         AS5047UDefs::DAECSource::ANGLEUNC);
}

template <AS5047UFrames::Frame ReadFrame>
bool AS5047U<ReadFrame>::setDAECStatus(AS5047UDefs::DAECompensation daecOn,
                                       AS5047UDefs::DAECSource daecSource)
{
    typename ReadFrame::Response reading = readRegister(
        AS5047UDefs::Registers::SETTINGS2, SensorErrors::COMMAND_FAILED);
    if (!reading.okCRC || reading.error)
        return false;

    uint16_t settings2 = reading.data & 0b10101111;  // leave the rest unchanged
    settings2 |= (static_cast<uint16_t>(daecOn) & 0b1 << 4);
    settings2 |= (static_cast<uint16_t>(daecSource) & 0b1 << 6);

    config.daecSource = daecSource;
    config.daecOn     = daecOn;
    lastError         = SensorErrors::NO_ERRORS;

    return writeAndCheck(AS5047UDefs::Registers::SETTINGS2, settings2,
                         SensorErrors::COMMAND_FAILED);
}

template class AS5047U<AS5047UFrames::SPIFrame16>;
template class AS5047U<AS5047UFrames::SPIFrame24>;

}  // namespace Boardcore
