/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include "H3LIS331DL.h"

namespace Boardcore
{

H3LIS331DL::H3LIS331DL(SPIBusInterface& spiBus, miosix::GpioPin cs,
                       SPIBusConfig cfg, H3LIS331DLDefs::OutputDataRate odr,
                       H3LIS331DLDefs::BlockDataUpdate bdu,
                       H3LIS331DLDefs::FullScaleRange fs)
    : spi(spiBus, cs, cfg), odr(odr), bdu(bdu), fs(fs), initialized(false)
{
    spi.config = getDefaultSPIConfig();
}

H3LIS331DL::H3LIS331DL(SPIBusInterface& spiBus, miosix::GpioPin cs,
                       H3LIS331DLDefs::OutputDataRate odr,
                       H3LIS331DLDefs::BlockDataUpdate bdu,
                       H3LIS331DLDefs::FullScaleRange fs)
    : H3LIS331DL(spiBus, cs, {}, odr, bdu, fs)
{
}

SPIBusConfig H3LIS331DL::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_128;
    spiConfig.mode         = SPI::Mode::MODE_3;
    spiConfig.byteOrder    = SPI::Order::LSB_FIRST;
    return spiConfig;
}

bool H3LIS331DL::init()
{
    if (initialized)
    {
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }

    SPITransaction spiTr(spi);

    uint8_t whoami =
        spiTr.readRegister(H3LIS331DLDefs::Registers::REG_WHO_AM_I);

    if (whoami != H3LIS331DLDefs::WHO_AM_I_ID)
    {
        lastError = SensorErrors::INVALID_WHOAMI;
        LOG_ERR(logger,
                "Failed init. Cause: INVALID_WHOAMI. Expected Value: "
                "{:X}. Actual Value: {:X}\n",
                H3LIS331DLDefs::WHO_AM_I_ID, whoami);
        return false;
    }

    // We assume everything is okay, if there are issues while writing the
    // configuration we will set this flag to false
    initialized = true;

    // CTRL_REG1 initialization
    {
        uint8_t ctrlReg1 = 0b0000'0000;  // Default: poweroff

        ctrlReg1 = odr | H3LIS331DLDefs::CTRL_REG1_XEN |
                   H3LIS331DLDefs::CTRL_REG1_YEN |
                   H3LIS331DLDefs::CTRL_REG1_ZEN;

        spiTr.writeRegister(H3LIS331DLDefs::Registers::REG_CTRL_REG1, ctrlReg1);

        miosix::delayUs(10);

        uint8_t ctrlReg1OnChip =
            spiTr.readRegister(H3LIS331DLDefs::Registers::REG_CTRL_REG1);

        initialized = (ctrlReg1 == ctrlReg1OnChip);

        if (!initialized)
        {

            LOG_ERR(logger,
                    "Control Register 1 After init: {:X}, expected "
                    "value:{:X}",
                    ctrlReg1OnChip, ctrlReg1);
            return false;
        }
    }

    {
        // CTRL_REG4 initialization
        // CTRL_REG4 controls the BDU (@see H3LIS331DL::BlockDataUpdate) and
        // the FSR (@see H3LIS331DL::FullScaleRange).
        uint8_t ctrlReg4 = 0b0000'0000;

        ctrlReg4 = bdu | fs;

        spiTr.writeRegister(H3LIS331DLDefs::Registers::REG_CTRL_REG4, ctrlReg4);

        miosix::delayUs(10);

        uint8_t ctrlReg4OnChip =
            spiTr.readRegister(H3LIS331DLDefs::Registers::REG_CTRL_REG4);

        initialized = (ctrlReg4 == ctrlReg4OnChip);

        if (!initialized)
        {

            LOG_ERR(logger,
                    "Control Register 1 After init: {:X}, expected "
                    "value:{:X}",
                    ctrlReg4OnChip, ctrlReg4);
            return false;
        }
    }

    return initialized;
}

bool H3LIS331DL::selfTest() { return true; }

H3LIS331DLData H3LIS331DL::sampleImpl()
{
    if (!initialized)
    {
        lastError = SensorErrors::NOT_INIT;
        return lastSample;
    }

    // Timestamp of the last sample
    uint64_t lastSampleTimestamp = TimestampTimer::getTimestamp();

    float x = 0;
    float y = 0;
    float z = 0;

    // Read output data registers (X, Y, Z)
    {
        SPITransaction spiTr(spi);
        uint8_t buff[7];

        spiTr.readRegisters(H3LIS331DLDefs::Registers::REG_STATUS_REG |
                                H3LIS331DLDefs::AUTOINC_ADDR,
                            buff, 7);

        // The status register that tells if new data is available is the first
        // byte that was read from the 7 byte read.
        uint8_t status = buff[0];
        uint16_t regX  = buff[2] << 8 | buff[1];
        uint16_t regY  = buff[4] << 8 | buff[3];
        uint16_t regZ  = buff[6] << 8 | buff[5];

        if (!(status & H3LIS331DLDefs::STATUS_REG_XYZDR))
        {
            lastError = SensorErrors::NO_NEW_DATA;
            LOG_DEBUG(logger, "No new data available.");
            return lastSample;
        }

        // Here we get the sensitivity based on the FullScaleRange
        float sensitivity = H3LIS331DLDefs::SENSITIVITY_VALUES[fs >> 4];

        int16_t xInt = static_cast<int16_t>(regX);
        float xFloat = static_cast<float>(xInt >> 4);
        x            = xFloat * sensitivity;

        int16_t yInt = static_cast<int16_t>(regY);
        float yFloat = static_cast<float>(yInt >> 4);
        y            = yFloat * sensitivity;

        int16_t zInt = static_cast<int16_t>(regZ);
        float zFloat = static_cast<float>(zInt >> 4);
        z            = zFloat * sensitivity;
    }

    return H3LIS331DLData(lastSampleTimestamp, x, y, z);
}

}  // namespace Boardcore
