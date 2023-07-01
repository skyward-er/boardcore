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

#include "ADS131M08.h"

#include <drivers/timer/TimestampTimer.h>
#include <util/crc16.h>
#include <utils/CRC.h>

using namespace Boardcore::ADS131M08RegisterBitMasks;

namespace Boardcore
{

ADS131M08::ADS131M08(SPIBusInterface &bus, miosix::GpioPin cs,
                     SPIBusConfig config)
    : ADS131M08(SPISlave(bus, cs, config))
{
}

ADS131M08::ADS131M08(SPISlave spiSlave) : spiSlave(spiSlave)
{
    // Reset the configuration
    channelsPGAGain[0] = PGA::PGA_1;
    channelsPGAGain[1] = PGA::PGA_1;
    channelsPGAGain[2] = PGA::PGA_1;
    channelsPGAGain[3] = PGA::PGA_1;
    channelsPGAGain[4] = PGA::PGA_1;
    channelsPGAGain[5] = PGA::PGA_1;
    channelsPGAGain[6] = PGA::PGA_1;
    channelsPGAGain[7] = PGA::PGA_1;
}

SPIBusConfig ADS131M08::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;
    spiConfig.mode         = SPI::Mode::MODE_1;
    return spiConfig;
}

void ADS131M08::setOversamplingRatio(OversamplingRatio ratio)
{
    changeRegister(Registers::REG_CLOCK, static_cast<uint16_t>(ratio),
                   REG_CLOCK_OSR);
}

bool ADS131M08::init()
{
    if (!reset())
    {
        lastError = SensorErrors::INIT_FAIL;
        LOG_ERR(logger, "Initialization failed");
        return false;
    }

    return true;
}

bool ADS131M08::reset()
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
        lastError = SensorErrors::COMMAND_FAILED;
        LOG_ERR(logger, "Reset command failed, response was {:X}", response);
        return false;
    }

    return true;
}

void ADS131M08::calibrateOffset()
{
    int32_t averageValues[8] = {0};

    // Sample the channels and average the samples
    for (int i = 0; i < 1000; i++)
    {
        sample();
        miosix::delayUs(1);

        for (int ii = 0; ii < 4; ii++)
            averageValues[ii] +=
                lastSample.voltage[ii] /
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
    writeRegister(Registers::REG_CH3_OCAL_MSB, averageValues[4] >> 8);
    writeRegister(Registers::REG_CH3_OCAL_LSB, averageValues[4] << 16);
    writeRegister(Registers::REG_CH3_OCAL_MSB, averageValues[5] >> 8);
    writeRegister(Registers::REG_CH3_OCAL_LSB, averageValues[5] << 16);
    writeRegister(Registers::REG_CH3_OCAL_MSB, averageValues[6] >> 8);
    writeRegister(Registers::REG_CH3_OCAL_LSB, averageValues[6] << 16);
    writeRegister(Registers::REG_CH3_OCAL_MSB, averageValues[7] >> 8);
    writeRegister(Registers::REG_CH3_OCAL_LSB, averageValues[7] << 16);
}

void ADS131M08::setChannelPGA(Channel channel, PGA gain)
{
    channelsPGAGain[static_cast<int>(channel)] = gain;

    if (channel <= Channel::CHANNEL_3)
    {
        int shift = static_cast<int>(channel) * 4;
        changeRegister(Registers::REG_GAIN_1,
                       static_cast<uint16_t>(gain) << shift,
                       REG_GAIN_PGAGAIN0 << shift);
    }
    else
    {
        int shift = (static_cast<int>(channel) - 4) * 4;
        changeRegister(Registers::REG_GAIN_2,
                       static_cast<uint16_t>(gain) << shift,
                       REG_GAIN_PGAGAIN0 << shift);
    }
}

void ADS131M08::setChannelOffset(Channel channel, uint32_t offset)
{
    // Set the correct registers based on the selected channel
    Registers regMSB, regLSB;
    switch (static_cast<int>(channel))
    {
        case 0:
            regMSB = Registers::REG_CH0_OCAL_MSB;
            regLSB = Registers::REG_CH0_OCAL_LSB;
            break;
        case 1:
            regMSB = Registers::REG_CH1_OCAL_MSB;
            regLSB = Registers::REG_CH1_OCAL_LSB;
            break;
        case 2:
            regMSB = Registers::REG_CH2_OCAL_MSB;
            regLSB = Registers::REG_CH2_OCAL_LSB;
            break;
        case 3:
            regMSB = Registers::REG_CH3_OCAL_MSB;
            regLSB = Registers::REG_CH3_OCAL_LSB;
            break;
        case 4:
            regMSB = Registers::REG_CH4_OCAL_MSB;
            regLSB = Registers::REG_CH4_OCAL_LSB;
            break;
        case 5:
            regMSB = Registers::REG_CH5_OCAL_MSB;
            regLSB = Registers::REG_CH5_OCAL_LSB;
            break;
        case 6:
            regMSB = Registers::REG_CH6_OCAL_MSB;
            regLSB = Registers::REG_CH6_OCAL_LSB;
            break;
        default:
            regMSB = Registers::REG_CH7_OCAL_MSB;
            regLSB = Registers::REG_CH7_OCAL_LSB;
            break;
    }

    writeRegister(regMSB, static_cast<uint16_t>(offset) >> 16);
    writeRegister(regLSB, static_cast<uint16_t>(offset) << 8);
}

void ADS131M08::setChannelGainCalibration(Channel channel, double gain)
{
    // Check gain value
    if (gain < 0 || gain > 2)
        return;

    // Set the correct registers based on the selected channel
    Registers regMSB, regLSB;
    switch (static_cast<int>(channel))
    {
        case 0:
            regMSB = Registers::REG_CH0_GCAL_MSB;
            regLSB = Registers::REG_CH0_GCAL_LSB;
            break;
        case 1:
            regMSB = Registers::REG_CH1_GCAL_MSB;
            regLSB = Registers::REG_CH1_GCAL_LSB;
            break;
        case 2:
            regMSB = Registers::REG_CH2_GCAL_MSB;
            regLSB = Registers::REG_CH2_GCAL_LSB;
            break;
        case 3:
            regMSB = Registers::REG_CH3_GCAL_MSB;
            regLSB = Registers::REG_CH3_GCAL_LSB;
            break;
        case 4:
            regMSB = Registers::REG_CH4_GCAL_MSB;
            regLSB = Registers::REG_CH4_GCAL_LSB;
            break;
        case 5:
            regMSB = Registers::REG_CH5_GCAL_MSB;
            regLSB = Registers::REG_CH5_GCAL_LSB;
            break;
        case 6:
            regMSB = Registers::REG_CH6_GCAL_MSB;
            regLSB = Registers::REG_CH6_GCAL_LSB;
            break;
        default:
            regMSB = Registers::REG_CH7_GCAL_MSB;
            regLSB = Registers::REG_CH7_GCAL_LSB;
            break;
    }

    // Compute the correct gain register parameter
    uint32_t rawGain = gain * 8388608;  // 2^23

    writeRegister(regMSB, rawGain >> 16);
    writeRegister(regLSB, rawGain << 8);
}

void ADS131M08::enableChannel(Channel channel)
{
    changeRegister(Registers::REG_CLOCK, 1 << (static_cast<int>(channel) + 8),
                   1 << (static_cast<int>(channel) + 8));
}

void ADS131M08::disableChannel(Channel channel)
{
    changeRegister(Registers::REG_CLOCK, 0,
                   1 << (static_cast<int>(channel) + 8));
}

void ADS131M08::enableGlobalChopMode()
{
    changeRegister(Registers::REG_CFG, ADS131M08RegisterBitMasks::REG_CFG_GC_EN,
                   ADS131M08RegisterBitMasks::REG_CFG_GC_EN);
}

void ADS131M08::disableGlobalChopMode()
{
    changeRegister(Registers::REG_CFG, 0,
                   ADS131M08RegisterBitMasks::REG_CFG_GC_EN);
}

ADCData ADS131M08::getVoltage(Channel channel)
{
    return {lastSample.timestamp, static_cast<uint8_t>(channel),
            lastSample.voltage[static_cast<uint8_t>(channel)]};
}

bool ADS131M08::selfTest()
{
    // TODO
    return true;
}

ADS131M08Data ADS131M08::sampleImpl()
{
    // Send the NULL command and read response
    uint8_t data[30] = {0};

    data[0] = static_cast<uint16_t>(Commands::NULL_CMD) >> 8;
    data[1] = static_cast<uint16_t>(Commands::NULL_CMD);

    SPITransaction transaction(spiSlave);
    transaction.transfer(data, sizeof(data));

    // Extract each channel value
    int32_t rawValue[8];
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
    rawValue[4] = static_cast<uint32_t>(data[15]) << 16 |
                  static_cast<uint32_t>(data[16]) << 8 |
                  static_cast<uint32_t>(data[17]);
    rawValue[5] = static_cast<uint32_t>(data[18]) << 16 |
                  static_cast<uint32_t>(data[19]) << 8 |
                  static_cast<uint32_t>(data[20]);
    rawValue[6] = static_cast<uint32_t>(data[21]) << 16 |
                  static_cast<uint32_t>(data[22]) << 8 |
                  static_cast<uint32_t>(data[23]);
    rawValue[7] = static_cast<uint32_t>(data[24]) << 16 |
                  static_cast<uint32_t>(data[25]) << 8 |
                  static_cast<uint32_t>(data[26]);

    // Extract and verify the CRC
    uint16_t dataCrc =
        static_cast<uint32_t>(data[27]) << 8 | static_cast<uint32_t>(data[28]);
    uint16_t calculatedCrc = CRCUtils::crc16(data, sizeof(data) - 3);

    if (dataCrc != calculatedCrc)
    {
        lastError = SensorErrors::BUS_FAULT;
        LOG_ERR(logger, "Failed CRC check during sensor sampling");

        // Return and don't convert the corrupted data
        return lastSample;
    }

    // Set the two complement
    for (int i = 0; i < 8; i++)
    {
        // Check for the sign bit
        if (rawValue[i] & 0x800000)
            rawValue[i] |= 0xFF000000;  // Extend the sign bit
    }

    // Convert values
    ADS131M08Data adcData;
    adcData.timestamp = TimestampTimer::getTimestamp();
    for (int i = 0; i < 8; i++)
    {
        adcData.voltage[i] =
            rawValue[i] *
            PGA_LSB_SIZE[static_cast<uint16_t>(channelsPGAGain[i])];
    }

    return adcData;
}

uint16_t ADS131M08::readRegister(Registers reg)
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

void ADS131M08::writeRegister(Registers reg, uint16_t data)
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
        lastError = SensorErrors::COMMAND_FAILED;
        LOG_ERR(logger, "Write command failed, response was {:X}", response);
    }
}

void ADS131M08::changeRegister(Registers reg, uint16_t newValue, uint16_t mask)
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
