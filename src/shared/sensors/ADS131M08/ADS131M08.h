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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "ADS131M08Data.h"
#include "ADS131M08Defs.h"

namespace Boardcore
{

/**
 * @brief Driver for ADS131M08 8 simultaneous channels adc.
 *
 * The ADS131M08 is an eight-channel, simultaneously-sampling, 24-bit,
 * delta-sigma (ΔΣ), analog-to-digital converter (ADC). The individual ADC
 * channels can be independently configured depending on the sensor input. A
 * low-noise, programmable gain amplifier (PGA) provides gains ranging from 1
 * to 128 to amplify low-level signals.
 *
 * Each channel on the ADS131M08 contains a digital decimation filter that
 * demodulates the output of the ΔΣ modulators. Offset and gain calibration
 * registers can be programmed to automatically adjust output samples for
 * measured offset and gain errors.
 *
 * The device features a "global-chop mode" to reduce offset error and offset
 * drift inherent to the device due to mismatch in the internal circuitry to
 * very low levels. When global-chop mode is enabled by setting the GC_EN bit in
 * the GLOBAL_CHOP_CFG register, the device uses the conversion results from two
 * consecutive internal conversions taken with opposite input polarity to cancel
 * the device offset voltage.
 *
 * Each channel has a dedicated input multiplexer that controls which signals
 * are routed to the ADC channels:
 * - The analog input pins corresponding to the given channel
 * - AGND, which is helpfu for offset calibraton
 * - Positive DC test signal
 * - Negative DC test signal
 *
 * Each channel also feature an integrated programmable gain amplifier (PGA)
 * that provides gains of 1 to 128. Each channel has an independent PGA.
 *
 * The device communicates via SPI, the maximum allowed frequency is 25MHz.
 *
 * The ADC can work on 3 different power configurations depending on the clock
 * frequency:
 * - High-resulution: 250-32KSPS
 * - Low-power: 125-16KSPS
 * - Very-low-power: 62.5-8KSPS
 * You will probably have the ADC in high resolution mode.
 */
class ADS131M08 : public Sensor<ADS131M08Data>
{
public:
    ADS131M08(SPIBusInterface& bus, miosix::GpioPin cs,
              SPIBusConfig config = getDefaultSPIConfig());

    explicit ADS131M08(SPISlave spiSlave);

    /**
     * Constructs the default config for SPI Bus.
     *
     * @returns The default SPIBusConfig.
     */
    static SPIBusConfig getDefaultSPIConfig();

    bool init() override;

    bool reset();

    /**
     * @brief Samples each channel, averages the samples and applies offset
     * compensation in the device.
     */
    void calibrateOffset();

    /**
     * @brief changes the oversampling ratio.
     *
     * Keep in mind that after changing the oversampling ratio the device resets
     * the internal digital filter and needs some time to settle.
     * So if you immediately take a sample you'll read zeros.
     *
     * @warning This is especially important if you perform an offset
     * calibration right after changing the oversampling ratio.
     */
    void setOversamplingRatio(ADS131M08Defs::OversamplingRatio ratio);

    /**
     * @brief Sets the channel programmable gain amplifier.
     *
     * The programmable gain amplifier allows the ADC to measure low level
     * signals with the full resolution.
     */
    void setChannelPGA(ADS131M08Defs::Channel channel, ADS131M08Defs::PGA gain);

    /**
     * @brief Sets the channel offset.
     *
     * Note that the device offset is a 24bit two complement.
     */
    void setChannelOffset(ADS131M08Defs::Channel channel, uint32_t offset);

    /**
     * @brief Sets the channel gain calibration.
     *
     * @param gain Gain value between 0 and 2. Values outside this range will be
     * capped.
     */
    void setChannelGainCalibration(ADS131M08Defs::Channel channel, double gain);

    void enableChannel(ADS131M08Defs::Channel channel);

    void disableChannel(ADS131M08Defs::Channel channel);

    /**
     * @brief Enables the global chop mode.
     *
     * When global chop mode is enabled the ADC uses the conversion results from
     * two consecutive internal conversions taken with opposite input polarity
     * to cancel the device internal offset voltage.
     *
     * The drawback of this mode is that the sampling rate is reduced. The
     * conversion period is reduced because every time the device swaps the
     * input polarity, the internal filter is reset. The ADC then always takes
     * three internal conversions to produce one result.
     *
     * For more details see chapter 8.4.3.2 of the datasheet.
     */
    void enableGlobalChopMode();

    /**
     * @brief Disables the global chop mode.
     *
     * See enableGlobalChopMode() for more details on the global chop mode.
     */
    void disableGlobalChopMode();

    /**
     * @brief The self test samples internally connects each channel to known
     * test signals and verifies if the sampled values are in an expected range.
     *
     * @returns True if the self test is successful, false otherwise.
     */
    bool selfTest() override;

private:
    ADS131M08Data sampleImpl() override;

    void setChannelInput(ADS131M08Defs::Channel channel,
                         ADS131M08Defs::Input input);

    /**
     * setChannelPGS() implementation without saving the gain value in
     * the local variable.
     */
    void setChannelPGAImpl(ADS131M08Defs::Channel channel,
                           ADS131M08Defs::PGA gain);

    /**
     * setChannelOffset() implementation without saving the offset value in
     * the local variable.
     */
    void setChannelOffsetImpl(ADS131M08Defs::Channel channel, uint32_t offset);

    /**
     * setChannelGainCalibration() implementation without saving the gain value
     * in the local variable.
     */
    void setChannelGainCalibrationImpl(ADS131M08Defs::Channel channel,
                                       double gain);

    ADS131M08Defs::Register getChannelConfigRegister(
        ADS131M08Defs::Channel channel);

    ADS131M08Defs::Register getChannelOffsetRegisterMSB(
        ADS131M08Defs::Channel channel);

    ADS131M08Defs::Register getChannelOffsetRegisterLSB(
        ADS131M08Defs::Channel channel);

    ADS131M08Defs::Register getChannelGainRegisterMSB(
        ADS131M08Defs::Channel channel);

    ADS131M08Defs::Register getChannelGainRegisterLSB(
        ADS131M08Defs::Channel channel);

    /**
     * @brief Sends a NULL command, reads the channels samples and stores the
     * values in the given array.
     *
     * @returns Returns true if the CRC is correct, false otherwise.
     */
    bool readSamples(int32_t rawValues[ADS131M08Defs::CHANNELS_NUM]);

    uint16_t readRegister(ADS131M08Defs::Register reg);

    void writeRegister(ADS131M08Defs::Register reg, uint16_t data);

    void changeRegister(ADS131M08Defs::Register reg, uint16_t newValue,
                        uint16_t mask);

    void sendCommand(SPITransaction& transaction, ADS131M08Defs::Command cmd,
                     uint8_t data[ADS131M08Defs::FULL_FRAME_SIZE]);

    /**
     * @brief Given a gain value returns the conversion value for a raw data
     * sample.
     */
    float getLSBSizeFromGain(ADS131M08Defs::PGA gain);

    SPISlave spiSlave;

    // Saving the current configuration of the device
    // This is necessary because the selfTest and calibrateOffset functions
    // temporarily resets the channels configuration
    ADS131M08Defs::PGA channelsPGAGain[ADS131M08Defs::CHANNELS_NUM];
    uint32_t channelsOffset[ADS131M08Defs::CHANNELS_NUM];
    double channelsGain[ADS131M08Defs::CHANNELS_NUM];

    PrintLogger logger = Logging::getLogger("ads131m08");
};

}  // namespace Boardcore
