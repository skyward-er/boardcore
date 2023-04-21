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

constexpr float H3LIS331DL::SENSITIVITY_VALUES[];

H3LIS331DL::H3LIS331DL(SPIBusInterface& spiBus, miosix::GpioPin cs,
                       SPIBusConfig cfg, OutputDataRate odr,
                       BlockDataUpdate bdu, FullScaleRange fs)
    : spi(spiBus, cs, cfg), odr(odr), bdu(bdu), fs(fs), initialized(false)
{
}

H3LIS331DL::H3LIS331DL(SPIBusInterface& spiBus, miosix::GpioPin cs,
                       OutputDataRate odr, BlockDataUpdate bdu,
                       FullScaleRange fs)
    : H3LIS331DL(spiBus, cs, {}, odr, bdu, fs)
{
}

bool H3LIS331DL::init()
{
    if (initialized)
    {
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }

    SPITransaction spiTr(spi);

    uint8_t whoami = spiTr.readRegister(Registers::REG_WHO_AM_I);

    if (whoami != WHO_AM_I_ID)
    {
        lastError = SensorErrors::INVALID_WHOAMI;
        LOG_ERR(logger,
                "Failed init. Cause: INVALID_WHOAMI. Expected Value: "
                "{:X}. Actual Value: {:X}\n",
                WHO_AM_I_ID, whoami);
        return false;
    }

    // We assume everything is okay, if there are issues while writing the
    // configuration we will set this flag to false
    initialized = true;

    // CTRL_REG1 initialization
    {
        uint8_t ctrlReg1 = 0b0000'0000;  // Default: poweroff

        ctrlReg1 = odr | CTRL_REG1_XEN | CTRL_REG1_YEN | CTRL_REG1_ZEN;

        spiTr.writeRegister(Registers::REG_CTRL_REG1, ctrlReg1);

        initialized &= ctrlReg1 == spiTr.readRegister(Registers::REG_CTRL_REG1);

        LOG_DEBUG(logger,
                  "Control Register 1 After init: {:X}, expected "
                  "value:{:X}\n",
                  spiTr.readRegister(Registers::REG_CTRL_REG1), ctrlReg1);
    }

    {
        // CTRL_REG4 initialization
        // CTRL_REG4 controls the BDU (@see H3LIS331DL::BlockDataUpdate) and
        // the FSR (@see H3LIS331DL::FullScaleRange).
        uint8_t ctrlReg4 = 0b0000'0000;

        ctrlReg4 = bdu | fs;

        spiTr.writeRegister(Registers::REG_CTRL_REG4, ctrlReg4);

        initialized &= ctrlReg4 == spiTr.readRegister(Registers::REG_CTRL_REG4);
        LOG_DEBUG(logger,
                  "Control Register 4 After init: {:X}, expected "
                  "value: {:X}\n",
                  spiTr.readRegister(Registers::REG_CTRL_REG4), ctrlReg4);
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

        // Read the status register that tells if new data is available.
        // This will allow us to read only data that is new and reuse data
        // that didn't change.
        uint8_t status = spiTr.readRegister(Registers::REG_STATUS_REG);

        if (status == 0)
        {
            lastError = SensorErrors::NO_NEW_DATA;
            return lastSample;  // No new data available
        }

        uint16_t lPart;  // data's LSB.
        uint16_t hPart;  // data's MSB.

        // Here we get the sensitivity based on the FullScaleRange
        float sensitivity = SENSITIVITY_VALUES[this->fs >> 4];

        // Read x-axis if new data is available or old data that was not
        // read was overrun and never read
        if (status & (STATUS_REG_XDR | STATUS_REG_XYZDR | STATUS_REG_XOR |
                      STATUS_REG_XYZOR))
        {

            // NOTE: Reading multiple bits with readRegisters or readRegister16
            // does not work
            lPart = spiTr.readRegister(Registers::REG_OUT_X_L);
            hPart = spiTr.readRegister(Registers::REG_OUT_X_H);

            // To convert the values I am casting the two 8 bits registers
            // into a signed 16 bit integer then I right-shift it by 4 to
            // match the 12 bit representation of the sensor. As the values
            // are casted into a signed int the shift also does sign
            // extention automatically.
            int16_t xInt = static_cast<int16_t>(lPart | (hPart << 8));
            // int16_t xInt = static_cast<int16_t>(
            //    spiTr.readRegister16(Registers::REG_OUT_X));
            float xFloat = static_cast<float>(xInt >> 4);
            x            = xFloat * sensitivity;
        }
        else  // else just use the last sample
        {
            x = lastSample.accelerationX;
        }

        // Read y-axis if new data is available or old data that was not
        // read was overrun and never read
        if (status & (STATUS_REG_YDR | STATUS_REG_XYZDR | STATUS_REG_YOR |
                      STATUS_REG_XYZOR))
        {
            // NOTE: Reading multiple bits with readRegisters or readRegister16
            // does not work
            lPart        = spiTr.readRegister(Registers::REG_OUT_Y_L);
            hPart        = spiTr.readRegister(Registers::REG_OUT_Y_H);
            int16_t yInt = static_cast<int16_t>(lPart | (hPart << 8));
            // int16_t yInt = static_cast<int16_t>(
            //    spiTr.readRegister16(Registers::REG_OUT_Y));
            float yFloat = static_cast<float>(yInt >> 4);
            y            = yFloat * sensitivity;
        }
        else
        {
            y = lastSample.accelerationY;
        }

        // Read z-axis if new data is available or old data that was not
        // read was overrun and never read
        if (status & (STATUS_REG_ZDR | STATUS_REG_XYZDR | STATUS_REG_ZOR |
                      STATUS_REG_XYZOR))
        {
            // NOTE: Reading multiple bits with readRegisters or readRegister16
            // does not work
            lPart        = spiTr.readRegister(Registers::REG_OUT_Z_L);
            hPart        = spiTr.readRegister(Registers::REG_OUT_Z_H);
            int16_t zInt = static_cast<int16_t>(lPart | (hPart << 8));
            // int16_t zInt = static_cast<int16_t>(
            //    spiTr.readRegister16(Registers::REG_OUT_Z));
            float zFloat = static_cast<float>(zInt >> 4);
            z            = zFloat * sensitivity;
        }
        else
        {
            z = lastSample.accelerationZ;
        }
    }

    return H3LIS331DLData(lastSampleTimestamp, x, y, z);
}

}  // namespace Boardcore
