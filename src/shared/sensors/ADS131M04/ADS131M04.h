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

#include "ADS131M04Data.h"
#include "ADS131M04Defs.h"

namespace Boardcore
{

/**
 * @brief Driver for ADS131M04 4 simultaneous channels adc.
 *
 * The ADS131M04 is a four-channel, simultaneously-sampling, 24-bit,
 * delta-sigma (ΔΣ), analog-to-digital converter (ADC). The individual ADC
 * channels can be independently configured depending on the sensor input. A
 * low-noise, programmable gain amplifier (PGA) provides gains ranging from 1
 * to 128 to amplify low-level signals.
 *
 * Each channel on the ADS131M04 contains a digital decimation filter that
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
class ADS131M04 : public Sensor<ADS131M04Data>
{
public:
    ADS131M04(SPIBusInterface& bus, miosix::GpioPin cs,
              SPIBusConfig config = getDefaultSPIConfig());

    explicit ADS131M04(SPISlave spiSlave);

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

    void setOversamplingRatio(ADS131M04Defs::OversamplingRatio ratio);

    void setChannelPGA(ADS131M04Defs::Channel channel, ADS131M04Defs::PGA gain);

    /**
     * @brief Sets the channel offset.
     *
     * Note that the device offset is a 24bit two complement.
     */
    void setChannelOffset(ADS131M04Defs::Channel channel, uint32_t offset);

    /**
     * @brief Sets the channel gain calibration.
     *
     * The ADS131M04 corrects for gain errors by multiplying the ADC conversion
     * result using the gain calibration registers.
     * The gain calibration value is interpreted as a 24bit unsigned. The values
     * corresponds to n * (1/2^23), ranging from 0 to 2 - (1/2^23).
     *
     * This function accepts a value between 0 and 2, it then compute the
     * correct gain register value.
     *
     * @param gain Must be between 0 and 2.
     */
    void setChannelGainCalibration(ADS131M04Defs::Channel channel, double gain);

    void enableChannel(ADS131M04Defs::Channel channel);

    void disableChannel(ADS131M04Defs::Channel channel);

    void enableGlobalChopMode();

    void disableGlobalChopMode();

    bool selfTest() override;

private:
    ADS131M04Data sampleImpl() override;

    void setChannelInput(ADS131M04Defs::Channel channel,
                         ADS131M04Defs::Input input);

    void setChannelPGAImpl(ADS131M04Defs::Channel channel,
                           ADS131M04Defs::PGA gain);

    void setChannelOffsetImpl(ADS131M04Defs::Channel channel, uint32_t offset);

    void setChannelGainCalibrationImpl(ADS131M04Defs::Channel channel,
                                       double gain);

    ADS131M04Defs::Register getChannelConfigRegister(
        ADS131M04Defs::Channel channel);

    ADS131M04Defs::Register getChannelOffsetRegisterMSB(
        ADS131M04Defs::Channel channel);

    ADS131M04Defs::Register getChannelOffsetRegisterLSB(
        ADS131M04Defs::Channel channel);

    ADS131M04Defs::Register getChannelGainRegisterMSB(
        ADS131M04Defs::Channel channel);

    ADS131M04Defs::Register getChannelGainRegisterLSB(
        ADS131M04Defs::Channel channel);

    bool readSamples(int32_t rawValues[ADS131M04Defs::CHANNELS_NUM]);

    uint16_t readRegister(ADS131M04Defs::Register reg);

    void writeRegister(ADS131M04Defs::Register reg, uint16_t data);

    void changeRegister(ADS131M04Defs::Register reg, uint16_t newValue,
                        uint16_t mask);

    void sendCommand(SPITransaction& transaction, ADS131M04Defs::Command cmd,
                     uint8_t data[ADS131M04Defs::FULL_FRAME_SIZE]);

    float getLSBSizeFromGain(ADS131M04Defs::PGA gain);

    SPISlave spiSlave;

    // Current channels configuration
    ADS131M04Defs::PGA channelsPGAGain[ADS131M04Defs::CHANNELS_NUM];
    uint32_t channelsOffset[ADS131M04Defs::CHANNELS_NUM];
    double channelsGain[ADS131M04Defs::CHANNELS_NUM];

    PrintLogger logger = Logging::getLogger("ads131m04");
};

}  // namespace Boardcore
