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
#include <util/crc16.h>
#include <utils/CRC.h>

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

bool ADS131M04::init()
{
    if (!reset())
    {
        last_error = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Initialization failed");
        return false;
    }

    return true;
}

bool ADS131M04::reset()
{
    // Write all the communication frame which is maximum 6 24bit words
    uint8_t data[18] = {0};

    // Prepare the command
    data[1] = static_cast<uint16_t>(Commands::RESET);

    SPITransaction transaction(spiSlave);
    transaction.write(data, sizeof(data));
    miosix::delayUs(10);
    transaction.read(data, 3);

    uint16_t response = data[0] << 8 | data[1];

    // Check for the correct response
    if (response != 0xFF24)
    {
        last_error = SensorErrors::COMMAND_FAILED;
        LOG_ERR(logger, "Reset command failed, response was {:X}", response);
        return false;
    }

    return true;
}

void ADS131M04::calibrateOffset()
{
    int32_t averageValues[4] = {0};

    // Sample the channels and average the samples
    for (int i = 0; i < 1000; i++)
    {
        sample();
        miosix::delayUs(1);

        for (int ii = 0; ii < 4; ii++)
            averageValues[ii] +=
                last_sample.voltage[ii] /
                PGA_LSB_SIZE[static_cast<uint16_t>(channelsPGAGain[ii])];
    }

    for (int i = 0; i < 4; i++)
        averageValues[i] /= 1000;

    // Configure the offset registers
    writeRegister(Registers::REG_CH0_OCAL_MSB, averageValues[0] >> 8);
    writeRegister(Registers::REG_CH0_OCAL_LSB, averageValues[0] << 16);
    writeRegister(Registers::REG_CH1_OCAL_MSB, averageValues[1] >> 8);
    writeRegister(Registers::REG_CH1_OCAL_LSB, averageValues[1] << 16);
    writeRegister(Registers::REG_CH2_OCAL_MSB, averageValues[2] >> 8);
    writeRegister(Registers::REG_CH2_OCAL_LSB, averageValues[2] << 16);
    writeRegister(Registers::REG_CH3_OCAL_MSB, averageValues[3] >> 8);
    writeRegister(Registers::REG_CH3_OCAL_LSB, averageValues[3] << 16);
}

void ADS131M04::setChannelPGA(Channel channel, PGA gain)
{
    channelsPGAGain[static_cast<int>(channel)] = gain;

    changeRegister(Registers::REG_GAIN,
                   static_cast<uint16_t>(gain)
                       << (static_cast<int>(channel) * 4),
                   REG_GAIN_PGAGAIN0 << (static_cast<int>(channel) * 4));
}

void ADS131M04::setChannelOffset(Channel channel, uint32_t offset)
{
    // Set the correct registers based on the selected channel
    Registers regMSB, regLSB;
    if (static_cast<int>(channel) == 0)
    {
        regMSB = Registers::REG_CH0_OCAL_MSB;
        regLSB = Registers::REG_CH0_OCAL_LSB;
    }
    else if (static_cast<int>(channel) == 1)
    {
        regMSB = Registers::REG_CH0_OCAL_MSB;
        regLSB = Registers::REG_CH0_OCAL_LSB;
    }
    else if (static_cast<int>(channel) == 1)
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

void ADS131M04::setChannelGainCalibration(Channel channel, double gain)
{
    // Check gain value
    if (gain < 0 || gain > 2)
        return;

    // Set the correct registers based on the selected channel
    Registers regMSB, regLSB;
    if (static_cast<int>(channel) == 0)
    {
        regMSB = Registers::REG_CH0_GCAL_MSB;
        regLSB = Registers::REG_CH0_GCAL_LSB;
    }
    else if (static_cast<int>(channel) == 1)
    {
        regMSB = Registers::REG_CH0_GCAL_MSB;
        regLSB = Registers::REG_CH0_GCAL_LSB;
    }
    else if (static_cast<int>(channel) == 1)
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

void ADS131M04::enableChannel(Channel channel)
{
    changeRegister(Registers::REG_CLOCK, 1 << (static_cast<int>(channel) + 8),
                   1 << (static_cast<int>(channel) + 8));
}

void ADS131M04::disableChannel(Channel channel)
{
    changeRegister(Registers::REG_CLOCK, 0,
                   1 << (static_cast<int>(channel) + 8));
}

void ADS131M04::enableGlobalChopMode()
{
    changeRegister(Registers::REG_CFG, ADS131M04RegisterBitMasks::REG_CFG_GC_EN,
                   ADS131M04RegisterBitMasks::REG_CFG_GC_EN);
}

void ADS131M04::disableGlobalChopMode()
{
    changeRegister(Registers::REG_CFG, 0,
                   ADS131M04RegisterBitMasks::REG_CFG_GC_EN);
}

bool ADS131M04::selfTest()
{
    // TODO
    return true;
}

ADS131M04Data ADS131M04::sampleImpl()
{
    // Send the NULL command and read response
    uint8_t data[18] = {0};

    data[0] = static_cast<uint16_t>(Commands::NULL_CMD) >> 8;
    data[1] = static_cast<uint16_t>(Commands::NULL_CMD);

    SPITransaction transaction(spiSlave);
    transaction.transfer(data, sizeof(data));

    // Extract each channel value
    int32_t rawValue[4];
    rawValue[0] = static_cast<uint32_t>(data[3]) << 16 |
                  static_cast<uint32_t>(data[4]) << 8 |
                  static_cast<uint32_t>(data[5]);
    rawValue[1] = static_cast<uint32_t>(data[6]) << 16 |
                  static_cast<uint32_t>(data[7]) << 8 |
                  static_cast<uint32_t>(data[8]);
    rawValue[2] = static_cast<uint32_t>(data[9]) << 16 |
                  static_cast<uint32_t>(data[10]) << 8 |
                  static_cast<uint32_t>(data[11]);
    rawValue[3] = static_cast<uint32_t>(data[12]) << 16 |
                  static_cast<uint32_t>(data[13]) << 8 |
                  static_cast<uint32_t>(data[14]);

    // Extract and verify the CRC
    uint16_t dataCrc =
        static_cast<uint32_t>(data[15]) << 8 | static_cast<uint32_t>(data[16]);
    uint16_t calculatedCrc = CRCUtils::crc16(data, sizeof(data) - 3);

    if (dataCrc != calculatedCrc)
    {
        last_error = SensorErrors::BUS_FAULT;
        LOG_ERR(logger, "Failed CRC check during sensor sampling");

        // Return and don't convert the corrupted data
        return last_sample;
    }

    // Set the two complement
    for (int i = 0; i < 4; i++)
    {
        // Check for the sign bit
        if (rawValue[i] & 0x800000)
            rawValue[i] |= 0xFF000000;  // Extend the sign bit
    }

    // Convert values
    ADS131M04Data adcData;
    adcData.timestamp = TimestampTimer::getTimestamp();
    for (int i = 0; i < 4; i++)
    {
        adcData.voltage[i] =
            rawValue[i] *
            PGA_LSB_SIZE[static_cast<uint16_t>(channelsPGAGain[i])];
    }

    return adcData;
}

uint16_t ADS131M04::readRegister(Registers reg)
{
    uint8_t data[3] = {0};

    // Prepare the command
    data[0] = static_cast<uint16_t>(Commands::RREG) >> 8 |
              static_cast<uint16_t>(reg) >> 1;
    data[1] = static_cast<uint16_t>(reg) << 7;

    SPITransaction transaction(spiSlave);
    transaction.write(data, sizeof(data));
    transaction.read(data, sizeof(data));

    return data[0] << 8 | data[1];
}

void ADS131M04::writeRegister(Registers reg, uint16_t data)
{
    uint8_t writeCommand[6] = {0};

    // Prepare the command
    writeCommand[0] = static_cast<uint16_t>(Commands::WREG) >> 8 |
                      static_cast<uint16_t>(reg) >> 1;
    writeCommand[1] = static_cast<uint16_t>(reg) << 7;
    writeCommand[3] = data >> 8;
    writeCommand[4] = data;

    SPITransaction transaction(spiSlave);
    transaction.write(writeCommand, sizeof(writeCommand));

    // Check response
    transaction.read(writeCommand, 3);
    uint16_t response = writeCommand[0] << 8 | writeCommand[1];
    if (response != (0x4000 | (static_cast<uint16_t>(reg) << 7)))
    {
        last_error = SensorErrors::COMMAND_FAILED;
        LOG_ERR(logger, "Write command failed, response was {:X}", response);
    }
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
