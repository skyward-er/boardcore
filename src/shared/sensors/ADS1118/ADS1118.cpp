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

#include "ADS1118.h"

#include <drivers/timer/TimestampTimer.h>
#include <interfaces/endianness.h>

namespace Boardcore
{

const ADS1118::ADS1118Config ADS1118::ADS1118_DEFAULT_CONFIG = {
    SINGLE_SHOT_MODE, FSR_2_048,  MUX_AIN0_AIN1, 0,     0,
    VALID_OPERATION,  PULL_UP_EN, ADC_MODE,      DR_128};

ADS1118::ADS1118(SPIBusInterface &bus, miosix::GpioPin cs,
                 ADS1118Config config_, SPIBusConfig spiConfig)
    : ADS1118(SPISlave(bus, cs, spiConfig), config_, false)
{
}

ADS1118::ADS1118(SPISlave spiSlave_, ADS1118Config config_, bool busyWait_,
                 int16_t tempDivider_)
    : spiSlave(spiSlave_), baseConfig(config_), busyWait(busyWait_),
      tempDivider(tempDivider_)

{
    // Initialize to 0 all the channels
    for (auto i = 0; i < NUM_OF_CHANNELS; i++)
    {
        channelsConfig[i].word = 0;

        // Set channel id for each ADS1118Data object in values array
        values[i].channelId = i;
    }

    // Reset the last written config value
    lastConfig.word = 0;
    lastConfigIndex = 0;
}

SPIBusConfig ADS1118::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_1;
    return spiConfig;
}

bool ADS1118::init() { return true; }

void ADS1118::enableInput(ADS1118Mux mux)
{
    enableInput(mux, baseConfig.bits.rate, baseConfig.bits.pga);
}

void ADS1118::enableInput(ADS1118Mux mux, ADS1118DataRate rate, ADS1118Pga pga)
{
    channelsConfig[mux].bits.mode       = baseConfig.bits.mode;
    channelsConfig[mux].bits.pga        = pga;
    channelsConfig[mux].bits.mux        = mux;
    channelsConfig[mux].bits.singleShot = 1;
    channelsConfig[mux].bits.noOp       = VALID_OPERATION;
    channelsConfig[mux].bits.pullUp     = baseConfig.bits.pullUp;
    channelsConfig[mux].bits.tempMode   = ADC_MODE;
    channelsConfig[mux].bits.rate       = rate;

    // Decrement the sample counter in order to read the temperature earlier on
    sampleCounter--;
}

void ADS1118::disableInput(ADS1118Mux mux) { channelsConfig[mux].word = 0; }

void ADS1118::disableAllInputs()
{
    for (auto i = 0; i < NUM_OF_CHANNELS; i++)
    {
        channelsConfig[i].word = 0;
    }
}

void ADS1118::enableTemperature()
{
    channelsConfig[TEMP_CHANNEL].word = TEMP_CONFIG;
}

void ADS1118::disableTemperature() { channelsConfig[TEMP_CHANNEL].word = 0; }

void ADS1118::enablePullUpResistor() { baseConfig.bits.pullUp = PULL_UP_EN; }

void ADS1118::disablePullUpResistor() { baseConfig.bits.pullUp = PULL_UP_DIS; }

void ADS1118::enableConfigCheck() { configCheck = true; }

void ADS1118::disableConfigCheck() { configCheck = false; }

ADS1118Data ADS1118::readInputAndWait(ADS1118Mux mux)
{
    readChannel(mux);
    return getVoltage(mux);
}

TemperatureData ADS1118::readTemperatureAndWait()
{
    readChannel(TEMP_CHANNEL);
    return getTemperature();
}

ADS1118Data ADS1118::getVoltage(ADS1118Mux mux) { return values[mux]; }

TemperatureData ADS1118::getTemperature()
{
    return TemperatureData{values[TEMP_CHANNEL].voltageTimestamp,
                           values[TEMP_CHANNEL].voltage};
}

int ADS1118::getConversionTime(int8_t channel)
{
    if (channel >= 0 && channel <= TEMP_CHANNEL)
    {
        return CONV_TIME[channelsConfig[channel].bits.rate];
    }
    else
    {
        return 0;
    }
}

bool ADS1118::selfTest()
{
    // Save the current configuration
    bool prevConfigCheck    = configCheck;
    uint16_t prevTempConfig = channelsConfig[TEMP_CHANNEL].word;

    // Enable temperature config in case it is not, just in case no other
    // configuration is enabled
    channelsConfig[TEMP_CHANNEL].word = TEMP_CONFIG;

    // Enable configuration check
    configCheck = true;

    // Check the communication by reading the temperature channel
    readChannel(TEMP_CHANNEL, INVALID_CHANNEL);

    // Restore the configuration
    configCheck                       = prevConfigCheck;
    channelsConfig[TEMP_CHANNEL].word = prevTempConfig;

    // Return false if an error occurred
    return lastError == 0;  // The sensor class misses a constant for no error
}

/**
 * The sampling of all the enabled inputs is performed in sequence starting from
 * the input with the lowest mux value.
 *
 * The first configuration is written and then, at the next call, read back the
 * result while transmitting the next configuration
 */
ADS1118Data ADS1118::sampleImpl()
{
    int8_t i = findNextEnabledChannel(lastConfigIndex + 1);

    // Write the next config and read the value (only if lastConfig is valid)
    readChannel(i, lastConfig.word != 0 ? lastConfigIndex : INVALID_CHANNEL);

    // Save index and config for the next read
    lastConfig.word = channelsConfig[i].word;
    lastConfigIndex = i;

    // Increment the sample counter
    sampleCounter++;

    // Regardless of the readChannel result, return the value stored
    return values[lastConfigIndex];
}

void ADS1118::readChannel(int8_t nextChannel, int8_t prevChannel)
{
    uint32_t writeData, transferData;

    lastError = NO_ERRORS;

    // Prepare the next configuration data
    if (nextChannel > INVALID_CHANNEL && nextChannel < NUM_OF_CHANNELS)
    {
        writeData = channelsConfig[nextChannel].word;
    }
    else
    {
        // A valid configuration will always be not equal to 0 since the valid
        // operation bits must be 0b01
        writeData = 0x0;
    }

    // Write next configuration and read previous value if necessary
    transferData = writeData;
    {
        SPITransaction transaction(spiSlave);
        transaction.transfer(reinterpret_cast<uint8_t *>(&transferData),
                             configCheck ? 4 : 2);
    }

    // If enabled and a valid configuration has just been written, check the
    // read back configuration
    if (configCheck && writeData)
    {
        // Compare the configuration with the second 16 bit word read
        if ((channelsConfig[nextChannel].word & CONFIG_MASK) !=
            (transferData >> 16 & CONFIG_MASK))
        {
            // Save the error
            lastError = BUS_FAULT;

            // Disable next value conversion
            lastConfig.word = 0;
        }
    }

    // Convert and save the value if last written configuration is valid
    if (prevChannel > INVALID_CHANNEL && prevChannel < NUM_OF_CHANNELS)
    {
        int16_t rawValue = swapBytes16(transferData);

        // TODO: the timestamp should be taken when the configuration is
        // written, now we could be reading the value after some time!
        values[prevChannel].voltageTimestamp = TimestampTimer::getTimestamp();

        if (prevChannel != 8)  // Voltage value
        {
            values[prevChannel].voltage =
                rawValue * PGA_LSB_SIZE[channelsConfig[prevChannel].bits.pga] /
                1000;
        }
        else  // Temperature value
        {
            values[TEMP_CHANNEL].voltage = (rawValue / 4) * TEMP_LSB_SIZE;
        }
    }
}

/**
 * To read an input we'll first write the appropriate configuration, wait for
 * the sample accordingly to the channel data rate and then read back the result
 */
void ADS1118::readChannel(int8_t channel)
{
    readChannel(channel, INVALID_CHANNEL);

    if (busyWait)
    {
        // Use a busy wait loop to be as precise as possible
        miosix::delayUs(getConversionTime(channel));
    }
    else
    {
        // Converto to milliseconds and increment by one to prevent premature
        // readings
        miosix::Thread::sleep(getConversionTime(channel) / 1000 + 1);
    }

    readChannel(INVALID_CHANNEL, channel);
}

/**
 * Search the next enabled channel starting from the specified channel and
 * returns it's index. If no channels are enabled return INVALID_CHANNEL
 */
int8_t ADS1118::findNextEnabledChannel(int8_t startChannel)
{
    int8_t &channel = startChannel;  // Just a change of name

    for (auto i = 0; i < 2; i++)
    {
        // Go to the first channel if channel is too big
        if (channel >= NUM_OF_CHANNELS)
        {
            channel = 0;
        }

        // Find next enabled mux config
        for (; channel < NUM_OF_CHANNELS && channelsConfig[channel].word == 0;
             channel++)
            ;

        // Check if the channel is valid and, for the temperature channel, if we
        // have to read it based on sampleCounter and tempDivider
        // If invalid try to search again starting from the first channel
        if (channel == TEMP_CHANNEL && sampleCounter % tempDivider != 0)
        {
            continue;
        }
        if (channel < NUM_OF_CHANNELS)
        {
            return channel;
        }
    }

    // If no valid channel has been fount return an invalid channel
    return INVALID_CHANNEL;
}

}  // namespace Boardcore
