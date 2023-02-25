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

#include <math.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>

#include <utils/Debug.h>

namespace Boardcore
{

constexpr float H3LIS331DL::SENSITIVITY_VALUES[];

bool H3LIS331DL::init()
{
    if (this->initialized)
    {
        lastError = SensorErrors::ALREADY_INIT;
        return false;
    }
    SPITransaction spiTr(this->spi);

    uint8_t whoami = spiTr.readRegister(Registers::WHO_AM_I);

    if (whoami != WHO_AM_I_ID)
    {
        lastError = SensorErrors::INVALID_WHOAMI;
        TRACE(
            "[H3LIS331DL] Failed init. Cause: INVALID_WHOAMI. Expected Value: "
            "0x%02x. Actual Value: 0x%02x\n",
            WHO_AM_I_ID, whoami);
        return false;
    }

    {                                     // CTRL_REG1 initialization
        uint8_t ctrlReg1  = 0b0000'0000;  // Default: poweroff
        uint8_t powerMode = 0;            // Remain in poweroff
        uint8_t dr        = 0x0;  // Data rate is 0 in case of low power mode

        if (this->odr <= OutputDataRate::ODR_LP_10)
        {
            // the low power mode output data rate is set in PM bits in the
            // CTRL_REG1 instead in the DR bytes and it starts at configuration
            // 010. Configuration 000 and 001 are used (in order) as poweroff
            // and normal power mode.
            powerMode = 0b010 + this->odr;
        }
        else
        {
            powerMode = 0b001;  // powermode is set to normal.

            // As ODR_50 (and the following) does not start at 0 I need to
            // subtract ODR_50 to align everything to 0
            dr = this->odr - OutputDataRate::ODR_50;
        }

        setBits(ctrlReg1, /* left shift */ 5, /*bitmask*/ 0b1110'0000,
                powerMode);
        setBits(ctrlReg1, 3, 0b0001'1000, dr);
        setBits(ctrlReg1, 0, 0b0000'0111, 0b111);

        spiTr.writeRegister(Registers::CTRL_REG1, ctrlReg1);

        this->initialized &=
            ctrlReg1 == spiTr.readRegister(Registers::CTRL_REG1);
        TRACE("[H3LIS331DL] Control Register 1 After init: 0x%02x\n", ctrlReg1);
    }

    {
        // CTRL_REG4 initialization
        // CTRL_REG4 controls the BDU (@see H3LIS331DL::BlockDataUpdate) and the
        // FSR (@see H3LIS331DL::FullScaleRange).
        uint8_t ctrlReg4 = 0b0000'0000;
        setBits(ctrlReg4, 7, 0b1000'0000, this->bdu);
        setBits(ctrlReg4, 4, 0b0011'0000, this->fs);

        spiTr.writeRegister(Registers::CTRL_REG4, ctrlReg4);

        this->initialized &=
            ctrlReg4 == spiTr.readRegister(Registers::CTRL_REG4);
        TRACE("[H3LIS331DL] Control Register 4 After init: 0x%02x\n", ctrlReg4);
    }

    return this->initialized;
}

H3LIS331DLData H3LIS331DL::sampleImpl()
{
    if (!this->initialized)
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
        SPITransaction spiTr(this->spi);

        // Read the status register that tells if new data is available.
        // This will allow us to read only data that is new and reuse data that
        // didn't change.
        uint8_t status = spiTr.readRegister(Registers::STATUS_REG);

        if ((status & 0b0000'1111) == 0)
        {
            lastError = SensorErrors::NO_NEW_DATA;
            return lastSample;  // No new data available
        }

        uint16_t lPart = 0;  // data's LSB.
        uint16_t hPart = 0;  // data's MSB.

        // Here we get the sensitivity based on the FullScaleRange
        float sensitivity = SENSITIVITY_VALUES[this->fs];

        // Read x-axis if new data is available or old data that was not read
        // was overrun and never read
        if (status & 0b0000'1001 || status & 0b0001'0000)
        {

            // NOTE: Reading multiple bits with readRegisters does not work
            lPart = spiTr.readRegister(Registers::OUT_X);
            hPart = spiTr.readRegister(Registers::OUT_X + 1);

            // To convert the values I am casting the two 8 bits registers into
            // a signed 16 bit integer then I right-shift it by 4 to match the
            // 12 bit representation of the sensor.
            // As the values are casted into a signed int the shift also does
            // sign extention automatically.
            int16_t xInt = static_cast<int16_t>(lPart | (hPart << 8));
            float xFloat = static_cast<float>(xInt >> 4);
            x            = xFloat * sensitivity;
        }
        else  // else just use the last sample
        {
            x = lastSample.accelerationX;
        }

        // Read y-axis if new data is available or old data that was not read
        // was overrun and never read
        if (status & 0b0000'1010 || status & 0b0010'0000)
        {
            hPart = lPart = 0;

            // NOTE: Reading multiple bits with readRegisters does not work
            lPart        = spiTr.readRegister(Registers::OUT_Y);
            hPart        = spiTr.readRegister(Registers::OUT_Y + 1);
            int16_t yInt = static_cast<int16_t>(lPart | (hPart << 8));
            float yFloat = static_cast<float>(yInt >> 4);
            y            = yFloat * sensitivity;
        }
        else
        {
            y = lastSample.accelerationY;
        }

        // Read z-axis if new data is available or old data that was not read
        // was overrun and never read
        if (status & 0b0000'1100 || status & 0b0100'0000)
        {
            hPart = lPart = 0;

            // NOTE: Reading multiple bits with readRegisters does not work
            lPart        = spiTr.readRegister(Registers::OUT_Z);
            hPart        = spiTr.readRegister(Registers::OUT_Z + 1);
            int16_t zInt = static_cast<int16_t>(lPart | (hPart << 8));
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

bool H3LIS331DL::selfTest() { return true; }

}  // namespace Boardcore
