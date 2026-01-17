/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Radu Raul
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

#include "AS5047DSPI.h"

#include <units/Angle.h>

namespace Boardcore
{

AS5047DSPI::AS5047DSPI(SPIBusInterface& spiBus, miosix::GpioPin cs,
                       SPIBusConfig spiCfg, AS5047DSPIConfig sensorCfg)
    : spiSlave{spiBus, cs, spiCfg}, initialized{false}, config{sensorCfg}
{
}

AS5047DSPI::AS5047DSPI(SPIBusInterface& spiBus, miosix::GpioPin cs,
                       AS5047DSPIConfig sensorCfg)
    : spiSlave{spiBus, cs}, initialized{false}, config{sensorCfg}
{
    spiSlave.config = getDefaultSPIConfig();
}

SPIBusConfig AS5047DSPI::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_128;
    spiConfig.mode         = SPI::Mode::MODE_1;
    spiConfig.byteOrder    = SPI::Order::MSB_FIRST;
    return spiConfig;
}

bool AS5047DSPI::init()
{
    if (initialized)
    {
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }

    // We assume everything is okay, if there are issues while writing the
    // configuration we will set this flag to false
    initialized = true;

    // CTRL_REG1 initialization
    {
        ReadResult reading;
        uint16_t settings1;

        reading = readRegister(AS5047DDefs::Registers::SETTINGS1);
        if (reading.hasError())
        {
            logReadRegisterError("Init1-SETTINGS1", reading.error);
            lastError = INIT_FAIL;
            return false;
        }

        settings1 &= 0x0003;  // The first two bits MUST NEVER BE CHANGED
        settings1 =
            (static_cast<uint16_t>(config.rotationDirection) << 2) |
            (static_cast<uint16_t>(config.dataInterface) << 3) |
            (static_cast<uint16_t>(config.daecEnabled) << 4) |
            ((static_cast<uint16_t>(config.abiResolution) & 0b1000) << 1) |
            (static_cast<uint16_t>(config.dataType) << 6) |
            (static_cast<uint16_t>(config.pwmEnabled) << 7);

        writeRegister(AS5047DDefs::Registers::SETTINGS1, settings1);
        reading = readRegister(AS5047DDefs::Registers::SETTINGS1);

        if (reading.hasError())
        {
            logReadRegisterError("Init2-SETTINGS1", reading.error);
            lastError = INIT_FAIL;
            return false;
        }

        initialized = reading.data == settings1;

        if (!initialized)
        {
            LOG_ERR(logger,
                    "SETTINGS1 register after init: {:X}, expected "
                    "value: {:X}.",
                    reading.data, settings1);
            dumpErrorRegister();
            lastError = SensorErrors::INIT_FAIL;
            return false;
        }
    }

    {
        ReadResult reading;
        uint16_t settings2;
        settings2 =
            static_cast<uint16_t>(config.uvwPolePairs) |
            (static_cast<uint16_t>(config.hystConfig) << 2) |
            ((static_cast<uint16_t>(config.abiResolution) & 0b0111) << 4);

        writeRegister(AS5047DDefs::Registers::SETTINGS2, settings2);
        reading = readRegister(AS5047DDefs::Registers::SETTINGS2);

        if (reading.hasError())
        {
            logReadRegisterError("SETTINGS2", reading.error);
            lastError = INIT_FAIL;
            return false;
        }

        initialized = reading.data == settings2;

        if (!initialized)
        {
            LOG_ERR(logger,
                    "SETTINGS2 register after init: {:X}, expected "
                    "value: {:X}.",
                    reading.data, settings2);
            dumpErrorRegister();
            lastError = SensorErrors::INIT_FAIL;
            return false;
        }
    }

    return initialized;
}

bool AS5047DSPI::selfTest() { return true; }

AS5047DData AS5047DSPI::sampleImpl()
{
    if (!initialized)
    {
        LOG_ERR(logger, "Sensor not initialized");
        lastError = SensorErrors::NOT_INIT;
        return lastSample;
    }

    // Timestamp of the last sample
    uint64_t lastSampleTimestamp = TimestampTimer::getTimestamp();

    // The angle reading is in the ANGLECOM register, these two additional
    // registers are for raw angle reading and for the correction that is
    // automatically done by the sensor. The correction can be disabled and this
    // will fill ANGLECOM with raw angle data, thus we don't really need to read
    // all three data registers MAG, ANGLEUNC and ANGLECOM but only ANGLECOM
    // also ANGLEUNC can be read by selecting data source via
    // Units::Angle::Degree cordicMagnitude = Units::Angle::Degree(0);
    // Units::Angle::Degree cordicAngle     = Units::Angle::Degree(0);
    {
        SPITransaction spiTr(spiSlave);
        Units::Angle::Degree daecAngle = Units::Angle::Degree(0);

        auto daecAngInt = readRegister(AS5047DDefs::Registers::ANGLECOM);
        if (daecAngInt.hasError())
        {
            logReadRegisterError("ANGLECOM", daecAngInt.error);
            lastError = COMMAND_FAILED;
            return;
        }

        daecAngle = Units::Angle::Degree(static_cast<float>(daecAngInt.data) *
                                         AS5047DDefs::SPI_ANGLE_RES);
        return AS5047DData(
            lastSampleTimestamp,
            static_cast<Units::Angle::Radian>(daecAngle).value());
    }
}

void AS5047DSPI::dumpErrorRegister()
{
    auto errReg = readRegister(AS5047DDefs::Registers::ERRFL);

    if (errReg.hasError())
        logReadRegisterError("ERROR_REGISTER", errReg.error);

    LOG_ERR(logger,
            "An error has occured, dumping error register. Parity "
            "Error: {}, "
            "Invalid Command: "
            "{}, Framing Error: {}",
            static_cast<bool>(errReg.data & 0b100),
            static_cast<bool>(errReg.data & 0b010),
            static_cast<bool>(errReg.data & 0b001));
}

void AS5047DSPI::setDAECStatus(AS5047DDefs::DAECStatus status)
{
    auto settings1Reading = readRegister(AS5047DDefs::Registers::SETTINGS1);
    auto settings1        = settings1Reading.data;

    if (settings1Reading.hasError())
    {
        logReadRegisterError("SETTINGS1", settings1Reading.error);
        lastError = COMMAND_FAILED;
        return;
    }

    settings1 &= AS5047DDefs::DAEC_SETTING_MASK;
    settings1 |= static_cast<uint16_t>(status) << AS5047DDefs::DAEC_SETTING_POS;

    writeRegister(AS5047DDefs::Registers::SETTINGS1, settings1);

    auto reading = readRegister(AS5047DDefs::Registers::SETTINGS1);

    if (reading.hasError())
    {
        logReadRegisterError("SETTINGS1", reading.error);
        lastError = COMMAND_FAILED;
        return;
    }

    if (reading.data != settings1)
    {
        LOG_ERR(logger,
                "Error while setting DAEC status. Expected {:X} but got {:X}",
                settings1, reading.data);
        dumpErrorRegister();
        lastError = COMMAND_FAILED;
        return;
    }

    config.daecEnabled = status;
    lastError          = NO_ERRORS;
}

void AS5047DSPI::setDataSource(AS5047DDefs::DataSelect dataSource)
{
    auto settings1Result = readRegister(AS5047DDefs::Registers::SETTINGS1);
    auto settings1       = settings1Result.data;
    if (settings1Result.error != AS5047DDefs::Error::NONE)
    {
        logReadRegisterError("SETTINGS1", settings1Result.error);
        lastError = SensorErrors::COMMAND_FAILED;
        return;
    }
    settings1 &= AS5047DDefs::DATASELECT_SETTING_MASK;
    settings1 |= static_cast<uint16_t>(dataSource)
                 << AS5047DDefs::DATASELECT_SETTING_POS;

    miosix::delayUs(AS5047DDefs::DELAY_BETWEEN_SPI_TRAN_US.value());
    writeRegister(AS5047DDefs::Registers::SETTINGS1, settings1);

    miosix::delayUs(AS5047DDefs::DELAY_BETWEEN_SPI_TRAN_US.value());
    auto result = readRegister(AS5047DDefs::Registers::SETTINGS1);

    if (result.error != AS5047DDefs::Error::NONE)
    {
        logReadRegisterError("SETTINGS1", result.error);
        lastError = COMMAND_FAILED;
        return;
    }

    if (result.data != settings1 || result.error != AS5047DDefs::Error::NONE)
    {
        LOG_ERR(logger,
                "Error while setting data source. Expected {:X} but got {:X}",
                settings1, result.data);
        dumpErrorRegister();
        lastError = SensorErrors::COMMAND_FAILED;
        return;
    }
    config.dataType = dataSource;
    lastError       = NO_ERRORS;
}

void AS5047DSPI::writeRegister(AS5047DDefs::Registers reg, uint16_t data)
{
    /*
     * The command frame is structured as such
     * Field  | bit position | Notes
     * Parity | 15           | Parity is calculated on the whole frame
     * R/W    | 14           | Read = 1, -> Write <- = 0
     * Data   | 13:0         | Actual Data
     *
     * It is used to access registers and also to send a NOP
     *
     * The data frame has a similar structure
     * Field  | bit position | Notes
     * Parity | 15           | Parity is calculated on the whole frame
     *    -   | 14           | Must be always set to 0
     * Data   | 13:0         | Actual Data
     *
     */

    uint16_t transaction[2] = {static_cast<uint16_t>(reg) & 0x3fff,
                               static_cast<uint16_t>(data) & 0x3fff};

    // we take out the 2 ms bits and by doing so both commmand and data
    // frame are ready for the transaction, the only missing thing is
    // calculating the parity

    transaction[0] |= getParity(transaction[0])
                      << AS5047DDefs::PARITY_BIT_POSITION;
    transaction[1] |= getParity(transaction[1])
                      << AS5047DDefs::PARITY_BIT_POSITION;

    SPITransaction spiTr{spiSlave};

    spiTr.write16(transaction, 4);
}

AS5047DSPI::ReadResult AS5047DSPI::readRegister(AS5047DDefs::Registers reg)
{
    /*
     * The command frame is structured as such
     * Field  | bit position | Notes
     * Parity | 15           | Parity is calculated on the whole frame
     * R/W    | 14           | -> Read <- = 1, Write = 0
     * Data   | 13:0         | Actual Data
     *
     * It is used to access registers and also to send a NOP
     *
     * The data frame has a similar structure
     * Field  | bit position | Notes
     * Parity | 15           | Parity is calculated on the whole frame
     * Error  | 14           | 1 if an error occured, 0 otherwise
     * Data   | 13:0         | Actual Data
     *
     */

    uint16_t transaction[2] = {
        static_cast<uint16_t>(reg) & 0x3fff,
        static_cast<uint16_t>(AS5047DDefs::Registers::NOP) & 0x3fff};

    // we take out the 2 ms bits and by doing so both commands are ready
    // we send a nop because the data is ready only after one SPI
    // transaction so we need to commands to get the data and the data is
    // available on the second command

    // before calculating parity we need to set both commands in read mode
    transaction[0] |= 0x4000;
    transaction[1] |= 0x4000;

    transaction[0] |= getParity(transaction[0])
                      << AS5047DDefs::PARITY_BIT_POSITION;
    transaction[1] |= getParity(transaction[1])
                      << AS5047DDefs::PARITY_BIT_POSITION;

    SPITransaction spiTr{spiSlave};

    spiTr.transfer16(transaction, 4);

    // now we can discard the first half word and check the parity on the
    // second one, if the parity of the whole frame (including the parity
    // bit) is 0 it means that the frame is likely correct

    transaction[0] =
        transaction[1] &
        0x3fff;  // here I'm discarding the first data frame by
                 // creating a copy of the second without the
                 // two msbits, this will be helpful in the next section

    if (getParity(transaction[1]))
        return ReadResult{transaction[0], AS5047DDefs::Error::PARITY_ERROR};

    // and after that we can check if the error bit is set
    if (transaction[1] & 0x4000)
        return ReadResult{transaction[0], AS5047DDefs::Error::OTHER_ERROR};

    return ReadResult{transaction[0], AS5047DDefs::Error::NONE};
}

uint16_t AS5047DSPI::getParity(uint16_t data)
{
    uint16_t parity = 0;
    for (size_t i = 0; i < sizeof(uint16_t) * 8; i++)
        parity ^= (data >> 0) & 0b1;
    return parity;
}

void AS5047DSPI::logReadRegisterError(std::string regName,
                                      AS5047DDefs::Error error)
{
    LOG_ERR(logger,
            "Error while reading {}. What happened? -> Received "
            "Frame Parity Error"
            ": {}, Other Error (see error register below): {}",
            regName, error == AS5047DDefs::Error::PARITY_ERROR,
            error == AS5047DDefs::Error::OTHER_ERROR);
    dumpErrorRegister();
}

}  // namespace Boardcore
