/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "ADS131M04.h"

#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore::ADS131M04RegisterBitMasks;

namespace Boardcore
{

ADS131M04::ADS131M04(SPISlave spiSlave) : spiSlave(spiSlave)
{
    // Reset the configuration
    channelsPGAGain[0] = PGA::PGA_1;
    channelsPGAGain[1] = PGA::PGA_1;
    channelsPGAGain[2] = PGA::PGA_1;
    channelsPGAGain[3] = PGA::PGA_1;
}

void ADS131M04::setOversamplingRatio(OversamplingRatio ratio)
{
    changeRegister(Registers::REG_CLOCK, static_cast<uint16_t>(ratio),
                   REG_CLOCK_OSR);
}

bool ADS131M04::init() { return true; }

template <int C>
void ADS131M04::setChannelPGA(PGA gain)
{
    static_assert(C >= 0 && C <= 3, "Channel must be between 0 and 3");

    changeRegister(Registers::REG_GAIN, static_cast<uint16_t>(gain) << (C * 4),
                   REG_GAIN_PGAGAIN0 << (C * 4));
}

template <int C>
void ADS131M04::setChannelOffset(uint32_t offset)
{
    static_assert(C >= 0 && C <= 3, "Channel must be between 0 and 3");

    // Set the correct registers based on the selected channel
    Registers regMSB, regLSB;
    if (C == 0)
    {
        regMSB = Registers::REG_CH0_OCAL_MSB;
        regLSB = Registers::REG_CH0_OCAL_LSB;
    }
    else if (C == 1)
    {
        regMSB = Registers::REG_CH0_OCAL_MSB;
        regLSB = Registers::REG_CH0_OCAL_LSB;
    }
    else if (C == 1)
    {
        regMSB = Registers::REG_CH0_OCAL_MSB;
        regLSB = Registers::REG_CH0_OCAL_LSB;
    }
    else
    {
        regMSB = Registers::REG_CH0_OCAL_MSB;
        regLSB = Registers::REG_CH0_OCAL_LSB;
    }

    writeRegister(regMSB, static_cast<uint16_t>(offset) >> 16);
    writeRegister(regLSB, static_cast<uint16_t>(offset) << 8);
}

template <int C>
void ADS131M04::setChannelGainCalibration(double gain)
{
    static_assert(C >= 0 && C <= 3, "Channel must be between 0 and 3");

    // Check gain value
    if (gain < 0 || gain > 2)
        return;

    // Set the correct registers based on the selected channel
    Registers regMSB, regLSB;
    if (C == 0)
    {
        regMSB = Registers::REG_CH0_GCAL_MSB;
        regLSB = Registers::REG_CH0_GCAL_LSB;
    }
    else if (C == 1)
    {
        regMSB = Registers::REG_CH0_GCAL_MSB;
        regLSB = Registers::REG_CH0_GCAL_LSB;
    }
    else if (C == 1)
    {
        regMSB = Registers::REG_CH0_GCAL_MSB;
        regLSB = Registers::REG_CH0_GCAL_LSB;
    }
    else
    {
        regMSB = Registers::REG_CH0_GCAL_MSB;
        regLSB = Registers::REG_CH0_GCAL_LSB;
    }

    // Compute the correct gain register parameter
    uint32_t rawGain = gain * 8388608;  // 2^23

    writeRegister(regMSB, rawGain >> 16);
    writeRegister(regLSB, rawGain << 8);
}

template <int C>
void ADS131M04::enableChannel()
{
    static_assert(C >= 0 && C <= 3, "Channel must be between 0 and 3");

    changeRegister(Registers::REG_CLOCK, 1 << (C + 8), 1 << (C + 8));
}

template <int C>
void ADS131M04::disableChannel()
{
    static_assert(C >= 0 && C <= 3, "Channel must be between 0 and 3");

    changeRegister(Registers::REG_CLOCK, 0 << (C + 8), 1 << (C + 8));
}

bool ADS131M04::selfTest() { return true; }

ADS131M04Data ADS131M04::sampleImpl()
{
    // Send the NULL command and read response
    uint16_t data[7] = {0};

    data[0] = static_cast<uint16_t>(Commands::NULL_CMD);

    SPITransaction transaction(spiSlave);
    transaction.transfer(reinterpret_cast<uint8_t*>(data), sizeof(data));

    // Extract each channel value
    int32_t rawValue[4];
    rawValue[0] = (uint32_t)data[1] << 8 | static_cast<uint32_t>(data[2]) >> 8;
    rawValue[1] = (uint32_t)(data[2] & 0xFF) << 16 | data[3];
    rawValue[2] = (uint32_t)data[4] << 8 | static_cast<uint32_t>(data[5]) >> 8;
    rawValue[3] = (uint32_t)(data[5] & 0xFF) << 16 | data[6];

    // Set the two complement
    for (int i = 0; i < 4; i++)
    {
        // Check for the sign bit
        if (rawValue[i] & 0x0800)
            rawValue[i] |= 0xF000;  // Extend the sign bit
    }

    // Convert values
    ADS131M04Data adcData;
    adcData.timestamp = TimestampTimer::getTimestamp();
    for (int i = 0; i < 4; i++)
    {
        adcData.voltage[i] = rawValue[i] * PGA_LSB_SIZE[i];
    }

    return adcData;
}

uint16_t ADS131M04::readRegister(Registers reg)
{
    // Prepare the command
    uint16_t readCommand = static_cast<uint16_t>(Commands::RREG) |
                           (static_cast<uint16_t>(reg) << 7);
    uint16_t regValue;

    SPITransaction transaction(spiSlave);
    transaction.write(readCommand);
    transaction.read(&regValue, 1);

    return regValue;
}

void ADS131M04::writeRegister(Registers reg, uint16_t data)
{
    // Prepare the command
    uint16_t writeCommand[2];
    writeCommand[0] = static_cast<uint16_t>(Commands::WREG) |
                      (static_cast<uint16_t>(reg) << 7);
    writeCommand[1] = data;

    SPITransaction transaction(spiSlave);
    transaction.write(writeCommand, 2);
}

void ADS131M04::changeRegister(Registers reg, uint16_t newValue, uint16_t mask)
{
    // Read the clock register
    uint16_t regValue = readRegister(reg);

    // Remove the OSR configuration
    regValue &= ~mask;

    // Set the OSR
    regValue |= newValue;

    // Write the new value
    writeRegister(reg, regValue);
}

}  // namespace Boardcore
