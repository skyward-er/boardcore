/* Copyright (c) 2023 Skyward Experimental Rocketry
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

using namespace miosix;
using namespace Boardcore::ADS131M08Defs;

namespace Boardcore
{

ADS131M08::ADS131M08(SPIBusInterface &bus, miosix::GpioPin cs,
                     SPIBusConfig spiConfig, const Config &config)
    : spiSlave(bus, cs, spiConfig), config(config)
{
    // Impose the correct spi mode
    spiSlave.config.mode = SPI::Mode::MODE_1;
}

bool ADS131M08::init()
{
    reset();
    applyConfig(config);

    return true;
}

bool ADS131M08::reset()
{
    SPITransaction transaction(spiSlave);

    uint8_t data[FULL_FRAME_SIZE] = {0};
    sendCommand(transaction, Command::RESET, data);

    // The reset command takes typically 5us to reload the register contents.
    // We wait 10us to be sure
    miosix::delayUs(10);

    // To read the response we need to read the full 3 bytes word. The response
    // code is contained in the first 2 bytes, the last byte is padding
    uint16_t response = transaction.read24() >> 8;

    // Check for the correct response
    if (response != RESET_CMD_RESPONSE)
    {
        lastError = SensorErrors::COMMAND_FAILED;
        LOG_ERR(logger, "Reset command failed, response was {:X}", response);
        return false;
    }

    return true;
}

void ADS131M08::applyConfig(Config config)
{
    for (int i = 0; i < CHANNELS_NUM; i++)
    {
        applyChannelConfig(static_cast<Channel>(i), config.channelsConfig[i]);
    }

    setOversamplingRatio(config.oversamplingRatio);
    if (config.globalChopModeEnabled)
    {
        enableGlobalChopMode();
    }
    else
    {
        disableGlobalChopMode();
    }

    // Save the newly applied configuration
    this->config = config;
}

void ADS131M08::calibrateOffset(Channel channel)
{
    // The device internal data chain firsts subtracts the offset and then
    // multiplies for the gain. To calibrate the offset we first reset it, then
    // take some samples and apply the average as the new offset.
    // So we need to reset the offset and gain
    Config::ChannelConfig calibrationConfig{
        .enabled = true,
        .pga     = PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0,
    };
    applyChannelConfig(channel, calibrationConfig);

    // Sample the channel and average the samples
    int32_t averageValue = 0;
    int realSampleCount  = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        // Wait for a sample to be ready
        Thread::sleep(4);

        // Sample the channels
        int32_t rawValues[CHANNELS_NUM];
        if (!readSamples(rawValues))
        {
            // If the CRC failed we skip this sample
            continue;
        }

        averageValue += rawValues[static_cast<int>(channel)];

        realSampleCount++;
    }

    if (realSampleCount == 0)
    {
        LOG_ERR(logger, "Calibration failed, no valid samples");
    }
    else
    {
        // Compute the average
        averageValue /= realSampleCount;
        LOG_INFO(logger, "Channel {} average offset: {}",
                 static_cast<int>(channel),
                 averageValue * getLSBSizeFromGain(calibrationConfig.pga));

        // Set the new offset only if valid, otherwise keep the old one
        if (averageValue != 0)
        {
            config.channelsConfig[static_cast<int>(channel)].offset =
                averageValue;
        }
    }

    // Reset the original configuration with the new offset value
    applyChannelConfig(channel,
                       config.channelsConfig[static_cast<int>(channel)]);
}

bool ADS131M08::selfTest()
{
    bool success = true;

    // Reset configuration and connect the positive test signal
    Config::ChannelConfig selfTestConfig{
        .enabled = true,
        .pga     = PGA::PGA_1,
        .offset  = 0,
        .gain    = 1.0,
    };
    for (int i = 0; i < CHANNELS_NUM; i++)
    {
        applyChannelConfig(static_cast<Channel>(i), selfTestConfig);
        setChannelInput(static_cast<Channel>(i), Input::POSITIVE_DC_TEST);
    }
    setOversamplingRatio(OversamplingRatio::OSR_16256);
    disableGlobalChopMode();

    // Take some samples
    int32_t averageValues[CHANNELS_NUM] = {0};
    int realSampleCount                 = 0;
    for (int i = 0; i < SELF_TEST_SAMPLES; i++)
    {
        // Wait for a sample to be ready
        Thread::sleep(4);

        // Sample the channels
        int32_t rawValues[CHANNELS_NUM];
        if (!readSamples(rawValues))
        {
            // If the CRC failed we skip this sample
            continue;
        }

        for (int j = 0; j < CHANNELS_NUM; j++)
        {
            averageValues[j] += rawValues[j];
        }

        realSampleCount++;
    }

    if (realSampleCount == 0)
    {
        LOG_ERR(logger,
                "Failed self test with positive DC signal, no valid samples");
        success = false;
    }
    else
    {
        // Check the values
        for (int i = 0; i < CHANNELS_NUM; i++)
        {
            // Compute the average
            averageValues[i] /= realSampleCount;

            // Convert the value to Volts
            float volts =
                averageValues[i] * getLSBSizeFromGain(selfTestConfig.pga);

            // Check if the value is in the acceptable range
            if (volts < V_REF * TEST_SIGNAL_FACTOR - TEST_SIGNAL_SLACK)
            {
                LOG_ERR(
                    logger,
                    "Self test failed on channel {} on positive test signal, "
                    "value was {}",
                    i, volts);
                success = false;
            }

            // Reset the raw value
            averageValues[i] = 0;
        }
    }

    // Connect all channels to the negative DC test signal
    for (int i = 0; i < CHANNELS_NUM; i++)
    {
        setChannelInput(static_cast<Channel>(i), Input::NEGATIVE_DC_TEST);
    }

    // Take some samples
    realSampleCount = 0;
    for (int i = 0; i < SELF_TEST_SAMPLES; i++)
    {
        // Wait for a sample to be ready
        Thread::sleep(4);

        // Sample the channels
        int32_t rawValues[CHANNELS_NUM];
        if (!readSamples(rawValues))
        {
            // If the CRC failed we skip this sample
            continue;
        }

        for (int j = 0; j < CHANNELS_NUM; j++)
        {
            averageValues[j] += rawValues[j];
        }

        realSampleCount++;
    }

    if (realSampleCount == 0)
    {
        LOG_ERR(logger,
                "Failed self test with positive DC signal, no valid samples");
        success = false;
    }
    else
    {
        // Check the values
        for (int i = 0; i < CHANNELS_NUM; i++)
        {
            // Compute the average
            averageValues[i] /= realSampleCount;

            // Convert the value to Volts
            float volts =
                averageValues[i] * getLSBSizeFromGain(selfTestConfig.pga);

            // Check if the value is in the acceptable range
            if (volts > -V_REF * TEST_SIGNAL_FACTOR + TEST_SIGNAL_SLACK)
            {
                LOG_ERR(
                    logger,
                    "Self test failed on channel {} on negative test signal, "
                    "value was {}",
                    i, volts);
                success = false;
            }
        }
    }

    // Reset channels input to default
    for (int i = 0; i < CHANNELS_NUM; i++)
    {
        setChannelInput(static_cast<Channel>(i), Input::DEFAULT);
    }

    // Reset to previous configuration
    applyConfig(config);

    // We fail even if one channel didn't pass the test
    return success;
}

ADS131M08Data ADS131M08::sampleImpl()
{
    // Read the samples and check if the CRC is correct
    int32_t rawValues[CHANNELS_NUM];
    if (!readSamples(rawValues))
    {
        // The CRC failed, return the last sample
        return lastSample;
    }

    // Convert the raw values to voltages
    ADS131M08Data adcData;
    adcData.timestamp = TimestampTimer::getTimestamp();
    for (int i = 0; i < CHANNELS_NUM; i++)
    {
        adcData.voltage[i] =
            rawValues[i] * getLSBSizeFromGain(config.channelsConfig[i].pga);
    }

    return adcData;
}

void ADS131M08::applyChannelConfig(Channel channel,
                                   Config::ChannelConfig config)
{
    if (config.enabled)
    {
        setChannelPGA(channel, config.pga);
        setChannelOffset(channel, config.offset);
        setChannelGain(channel, config.gain);
        enableChannel(channel);
    }
    else
    {
        disableChannel(channel);
    }
}

void ADS131M08::setOversamplingRatio(OversamplingRatio ratio)
{
    changeRegister(Register::REG_CLOCK, static_cast<uint16_t>(ratio),
                   RegClockMasks::OSR);
}

void ADS131M08::setChannelPGA(Channel channel, PGA gain)
{
    if (channel <= Channel::CHANNEL_3)
    {
        int shift = static_cast<int>(channel) * 4;
        changeRegister(Register::REG_GAIN_1,
                       static_cast<uint16_t>(gain) << shift,
                       RegGainMasks::PGA_GAIN_0 << shift);
    }
    else
    {
        int shift = (static_cast<int>(channel) - 4) * 4;
        changeRegister(Register::REG_GAIN_2,
                       static_cast<uint16_t>(gain) << shift,
                       RegGainMasks::PGA_GAIN_0 << shift);
    }
}

void ADS131M08::setChannelOffset(Channel channel, int32_t offset)
{
    // The offset is a 24 bit value. Its two most significant bytes are stored
    // in the MSB register, and the least significant in the LSB register, like
    // this:
    // +----------+-------+
    // |          | 23-16 |
    // | OCAL_MSB +-------+
    // |          | 15-8  |
    // +----------+-------+
    // |          |  7-0  |
    // | OCAL_LSB +-------+
    // |          |       |
    // +----------+-------+
    writeRegister(getChannelOffsetRegisterMSB(channel), offset >> 8);
    writeRegister(getChannelOffsetRegisterLSB(channel), offset << 8);
}

void ADS131M08::setChannelGain(Channel channel, double gain)
{
    // If the user passes a value outside the range [0, 2] we cap it.
    if (gain < 0)
    {
        gain = 0;
    }
    else if (gain > 2)
    {
        gain = 2;
    }

    // The ADS131M08 corrects for gain errors by multiplying the ADC conversion
    // result using the gain calibration registers.
    // The contents of the gain calibration registers are interpreted by the
    // dives as 24-bit unsigned values corresponding to linear steps from 0 to
    // 2-(1/2^23).
    // So effectively the register value is a fixed point number with 1bit for
    // the integer part and 23 bits for the fractional part.
    // So to convert the gain value to the register value we multiply it by 2^23
    uint32_t rawGain = gain * (1 << 23);

    // The gain is a 24 bit value. Its two most significant bytes are stored
    // in the MSB register, and the least significant in the LSB register, like
    // this:
    // +----------+-------+
    // |          | 23-16 |
    // | GCAL_MSB +-------+
    // |          | 15-8  |
    // +----------+-------+
    // |          |  7-0  |
    // | GCAL_LSB +-------+
    // |          |       |
    // +----------+-------+
    writeRegister(getChannelGainRegisterMSB(channel), rawGain >> 8);
    writeRegister(getChannelGainRegisterLSB(channel), rawGain << 8);
}

void ADS131M08::enableChannel(Channel channel)
{
    changeRegister(Register::REG_CLOCK, 1 << (static_cast<int>(channel) + 8),
                   1 << (static_cast<int>(channel) + 8));
}

void ADS131M08::disableChannel(Channel channel)
{
    changeRegister(Register::REG_CLOCK, 0,
                   1 << (static_cast<int>(channel) + 8));
}

void ADS131M08::enableGlobalChopMode()
{
    changeRegister(Register::REG_CFG, RegConfigurationMasks::GC_EN,
                   RegConfigurationMasks::GC_EN);
}

void ADS131M08::disableGlobalChopMode()
{
    changeRegister(Register::REG_CFG, 0, RegConfigurationMasks::GC_EN);
}

void ADS131M08::setChannelInput(Channel channel, Input input)
{
    Register reg = getChannelConfigRegister(channel);
    changeRegister(reg, static_cast<uint16_t>(input), RegChannelMasks::CFG_MUX);
}

Register ADS131M08::getChannelConfigRegister(Channel channel)
{
    switch (static_cast<int>(channel))
    {
        case 0:
            return Register::REG_CH0_CFG;
        case 1:
            return Register::REG_CH1_CFG;
        case 2:
            return Register::REG_CH2_CFG;
        case 3:
            return Register::REG_CH3_CFG;
        case 4:
            return Register::REG_CH4_CFG;
        case 5:
            return Register::REG_CH5_CFG;
        case 6:
            return Register::REG_CH6_CFG;
        default:
            return Register::REG_CH7_CFG;
    }
}

Register ADS131M08::getChannelOffsetRegisterMSB(Channel channel)
{
    switch (static_cast<int>(channel))
    {
        case 0:
            return Register::REG_CH0_OCAL_MSB;
        case 1:
            return Register::REG_CH1_OCAL_MSB;
        case 2:
            return Register::REG_CH2_OCAL_MSB;
        case 3:
            return Register::REG_CH3_OCAL_MSB;
        case 4:
            return Register::REG_CH4_OCAL_MSB;
        case 5:
            return Register::REG_CH5_OCAL_MSB;
        case 6:
            return Register::REG_CH6_OCAL_MSB;
        default:
            return Register::REG_CH7_OCAL_MSB;
    }
}

Register ADS131M08::getChannelOffsetRegisterLSB(Channel channel)
{
    switch (static_cast<int>(channel))
    {
        case 0:
            return Register::REG_CH0_OCAL_LSB;
        case 1:
            return Register::REG_CH1_OCAL_LSB;
        case 2:
            return Register::REG_CH2_OCAL_LSB;
        case 3:
            return Register::REG_CH3_OCAL_LSB;
        case 4:
            return Register::REG_CH4_OCAL_LSB;
        case 5:
            return Register::REG_CH5_OCAL_LSB;
        case 6:
            return Register::REG_CH6_OCAL_LSB;
        default:
            return Register::REG_CH7_OCAL_LSB;
    }
}

Register ADS131M08::getChannelGainRegisterMSB(Channel channel)
{
    switch (static_cast<int>(channel))
    {
        case 0:
            return Register::REG_CH0_GCAL_MSB;
        case 1:
            return Register::REG_CH1_GCAL_MSB;
        case 2:
            return Register::REG_CH2_GCAL_MSB;
        case 3:
            return Register::REG_CH3_GCAL_MSB;
        case 4:
            return Register::REG_CH4_GCAL_MSB;
        case 5:
            return Register::REG_CH5_GCAL_MSB;
        case 6:
            return Register::REG_CH6_GCAL_MSB;
        default:
            return Register::REG_CH7_GCAL_MSB;
    }
}

Register ADS131M08::getChannelGainRegisterLSB(Channel channel)
{
    switch (static_cast<int>(channel))
    {
        case 0:
            return Register::REG_CH0_GCAL_LSB;
        case 1:
            return Register::REG_CH1_GCAL_LSB;
        case 2:
            return Register::REG_CH2_GCAL_LSB;
        case 3:
            return Register::REG_CH3_GCAL_LSB;
        case 4:
            return Register::REG_CH4_GCAL_LSB;
        case 5:
            return Register::REG_CH5_GCAL_LSB;
        case 6:
            return Register::REG_CH6_GCAL_LSB;
        default:
            return Register::REG_CH7_GCAL_LSB;
    }
}

bool ADS131M08::readSamples(int32_t rawValues[CHANNELS_NUM])
{
    // Send the NULL command and read response
    uint8_t data[FULL_FRAME_SIZE] = {0};

    data[0] = (static_cast<uint16_t>(Command::NULL_CMD) & 0xff00) >> 8;
    data[1] = (static_cast<uint16_t>(Command::NULL_CMD) & 0x00ff);

    SPITransaction transaction(spiSlave);
    transaction.transfer(data, sizeof(data));

    // Extract and verify the CRC
    uint16_t dataCrc =
        static_cast<uint16_t>(data[27]) << 8 | static_cast<uint16_t>(data[28]);
    uint16_t calculatedCrc = CRCUtils::crc16(data, sizeof(data) - 3);

    if (dataCrc != calculatedCrc)
    {
        lastError = SensorErrors::BUS_FAULT;
        LOG_ERR(logger, "Failed CRC check during sensor sampling");

        // Return and don't convert the corrupted data
        return false;
    }

    // Extract each channel value
    for (int i = 0; i < CHANNELS_NUM; i++)
    {
        rawValues[i] = static_cast<uint32_t>(data[i * 3 + 3]) << 16 |
                       static_cast<uint32_t>(data[i * 3 + 4]) << 8 |
                       static_cast<uint32_t>(data[i * 3 + 5]);

        // Extend the sign bit with a double shift
        rawValues[i] <<= 8;
        rawValues[i] >>= 8;
    }

    return true;
}

uint16_t ADS131M08::readRegister(Register reg)
{
    uint8_t data[3] = {0};

    // Prepare the command
    data[0] = static_cast<uint16_t>(Command::RREG) >> 8 |
              static_cast<uint16_t>(reg) >> 1;
    data[1] = static_cast<uint16_t>(reg) << 7;

    SPITransaction transaction(spiSlave);
    transaction.write(data, sizeof(data));
    transaction.read(data, sizeof(data));

    return data[0] << 8 | data[1];
}

void ADS131M08::writeRegister(Register reg, uint16_t data)
{
    // The write command uses two communication words (3 bytes each), one for
    // the register address and one for the data to write
    uint8_t writeCommand[6] = {0};

    // Prepare the command
    writeCommand[0] = static_cast<uint16_t>(Command::WREG) >> 8 |
                      static_cast<uint16_t>(reg) >> 1;
    writeCommand[1] = static_cast<uint16_t>(reg) << 7;
    writeCommand[3] = data >> 8;
    writeCommand[4] = data;

    SPITransaction transaction(spiSlave);
    transaction.write(writeCommand, sizeof(writeCommand));

    // The response contains a fixed part and the register address.
    uint16_t response = transaction.read24() >> 8;
    if (response != (WRITE_CMD_RESPONSE | (static_cast<uint16_t>(reg) << 7)))
    {
        lastError = SensorErrors::COMMAND_FAILED;
        LOG_ERR(logger, "Write command failed, response was {:X}", response);
    }
}

void ADS131M08::changeRegister(Register reg, uint16_t newValue, uint16_t mask)
{
    // Read the clock register
    uint16_t regValue = readRegister(reg);

    // Remove the target configuration
    regValue &= ~mask;

    // Set the new value
    regValue |= newValue;

    // Write the new value
    writeRegister(reg, regValue);
}

void ADS131M08::sendCommand(SPITransaction &transaction, Command command,
                            uint8_t data[FULL_FRAME_SIZE])
{
    // All commands (a part from read and write) needs the full 10 words
    // communication frame. Each word is (by default) 3 bytes long

    // The command is 16 bits long and goes in the most significant bytes of the
    // first communication word
    data[0] = (static_cast<uint16_t>(command) & 0xff00) >> 8;
    data[1] = (static_cast<uint16_t>(command) & 0x00ff);

    transaction.write(data, FULL_FRAME_SIZE);
}

float ADS131M08::getLSBSizeFromGain(PGA gain)
{
    return PGA_LSB_SIZE[static_cast<uint16_t>(gain)];
}

}  // namespace Boardcore
