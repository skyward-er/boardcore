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
    : spi{spiBus, cs, spiCfg}, initialized{false}, config{sensorCfg}
{
}

AS5047DSPI::AS5047DSPI(SPIBusInterface& spiBus, miosix::GpioPin cs,
                       AS5047DSPIConfig sensorCfg)
    : spi{spiBus, cs}, initialized{false}, config{sensorCfg}
{
    spi.config = getDefaultSPIConfig();
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

    SPITransaction spiTr(spi);

    // We assume everything is okay, if there are issues while writing the
    // configuration we will set this flag to false
    initialized = true;

    // CTRL_REG1 initialization
    {
        uint16_t settings1;

        settings1 = spiTr.readRegister16(AS5047DDefs::Registers::SETTINGS1);

        settings1 &= 0x0003;  // The first two bits MUST NEVER BE CHANGED
        settings1 = (config.rotationDirection << 2) |
                    (config.dataInterface << 3) | (config.daecEnabled << 4) |
                    ((config.abiResolution & 0b1000) << 1) |
                    (config.dataType << 6) | (config.pwmEnabled << 7);

        miosix::delayUs(AS5047DDefs::DELAY_BETWEEN_SPI_TRAN_US.value());
        spiTr.writeRegister16(AS5047DDefs::Registers::SETTINGS1, settings1);
        miosix::delayUs(AS5047DDefs::DELAY_BETWEEN_SPI_TRAN_US.value());
        uint16_t data = spiTr.readRegister16(AS5047DDefs::Registers::SETTINGS1);

        initialized = data == settings1;

        if (!initialized)
        {
            LOG_ERR(logger,
                    "SETTINGS1 register after init: {:X}, expected "
                    "value: {:X}.",
                    data, settings1);
            dumpErrorRegister(spiTr);
            lastError = SensorErrors::INIT_FAIL;
            return false;
        }
    }

    {
        uint16_t settings2;
        settings2 = config.uvwPolePairs | (config.hystConfig << 2) |
                    ((config.abiResolution & 0b0111) << 4);

        spiTr.writeRegister16(AS5047DDefs::Registers::SETTINGS2, settings2);
        miosix::delayUs(AS5047DDefs::DELAY_BETWEEN_SPI_TRAN_US.value());
        uint16_t data = spiTr.readRegister16(AS5047DDefs::Registers::SETTINGS2);

        initialized = data == settings2;

        if (!initialized)
        {
            LOG_ERR(logger,
                    "SETTINGS2 register after init: {:X}, expected "
                    "value: {:X}.",
                    data, settings2);
            dumpErrorRegister(spiTr);
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
        SPITransaction spiTr(spi);
        Units::Angle::Degree daecAngle = Units::Angle::Degree(0);

        auto daecAngInt =
            spiTr.readRegister16(AS5047DDefs::Registers::ANGLECOM);

        daecAngle = Units::Angle::Degree(static_cast<float>(daecAngInt) *
                                         AS5047DDefs::SPI_ANGLE_RES);
        return AS5047DData(
            lastSampleTimestamp,
            static_cast<Units::Angle::Radian>(daecAngle).value());
    }
}

void AS5047DSPI::dumpErrorRegister(SPITransaction& spiTr)
{
    uint16_t errReg = spiTr.readRegister16(AS5047DDefs::Registers::ERRFL);

    LOG_ERR(logger,
            "An error has occured, dumping error register. Parity Error: {}, "
            "Invalid Command: "
            "{}, Framing Error: {}",
            static_cast<bool>(errReg & 0b100),
            static_cast<bool>(errReg & 0b010),
            static_cast<bool>(errReg & 0b001));
}

void AS5047DSPI::setDAECStatus(AS5047DDefs::DAECStatus status)
{
    SPITransaction spiTr(spi);

    auto settings1 = spiTr.readRegister16(AS5047DDefs::Registers::SETTINGS1);
    settings1 &= AS5047DDefs::DAEC_SETTING_MASK;
    settings1 |= status << AS5047DDefs::DAEC_SETTING_POS;

    miosix::delayUs(AS5047DDefs::DELAY_BETWEEN_SPI_TRAN_US.value());
    spiTr.writeRegister16(AS5047DDefs::Registers::SETTINGS1, settings1);

    miosix::delayUs(AS5047DDefs::DELAY_BETWEEN_SPI_TRAN_US.value());
    auto data = spiTr.readRegister16(AS5047DDefs::Registers::SETTINGS1);

    if (data != settings1)
    {
        LOG_ERR(logger,
                "Error while setting DAEC status. Expected {:X} but got {:X}",
                settings1, data);
        dumpErrorRegister(spiTr);
        return;
    }

    config.daecEnabled = status;
}

void AS5047DSPI::setDataSource(AS5047DDefs::DataSelect dataSource)
{
    SPITransaction spiTr(spi);

    auto settings1 = spiTr.readRegister16(AS5047DDefs::Registers::SETTINGS1);
    settings1 &= AS5047DDefs::DATASELECT_SETTING_MASK;
    settings1 |= dataSource << AS5047DDefs::DATASELECT_SETTING_POS;

    miosix::delayUs(AS5047DDefs::DELAY_BETWEEN_SPI_TRAN_US.value());
    spiTr.writeRegister16(AS5047DDefs::Registers::SETTINGS1, settings1);

    miosix::delayUs(AS5047DDefs::DELAY_BETWEEN_SPI_TRAN_US.value());
    auto data = spiTr.readRegister16(AS5047DDefs::Registers::SETTINGS1);

    if (data != settings1)
    {
        LOG_ERR(logger,
                "Error while setting data source. Expected {:X} but got {:X}",
                settings1, data);
        dumpErrorRegister(spiTr);
        return;
    }
    config.dataType = dataSource;
}

}  // namespace Boardcore
