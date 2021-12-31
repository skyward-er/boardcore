/* Copyright (c) 2015-2020 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Luca Erbetta, Alberto Nidasio
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

#include <miosix.h>
#include <sensors/Sensor.h>

#include "InternalADCData.h"

namespace Boardcore
{

/**
 * @brief Driver for stm32 internal ADC
 *
 * Allows conversions on multiple channels with per-channel sample time.
 *
 * The driver configures the ADC by itself, including the clock.
 *
 * This driver implements the following ADC's features:
 * - injected channels: a list of up to 4 channels that can be converted
 * - scan mode with regular channels: up to 16 channels converted in the same
 * sequence as they are enabled
 * - DMA: Used in scan mode to transfer data from the peripheral data register
 *
 * When you don't use the DMA, the driver works with the injected channels.
 * When a conversion is started the values are stored in 4 specific data
 * registes, whithout the need to use DMA.
 *
 * If you need more than 4 channels you must use DMA. When the regular group
 * of channels is converted, the data will be stored sequentially in a single
 * data register and DMA is needed to save the value between each conversion in
 * another memory location.
 *
 * When using DMA the driver will configure only the memory addresses, you must
 * configure the stream properly!
 *
 * Examples of how to use the internal ADC driver and set up the DMA stream
 * can be found in the test files (test-internal-adc*).
 *
 * The ADC has also other features as value offsets for injected channels,
 * interrupts (such as dma transfer and watchdog), conversion triggering with
 * timers and multi adc mode.
 * This features are not implemented to keep the driver simple.
 */
class InternalADC : public Sensor<ADCData>
{
public:
    /**
     * @brief ADC channels enumeration
     */
    enum Channel : uint8_t
    {
        CH0 = 0,
        CH1,
        CH2,
        CH3,
        CH4,
        CH5,
        CH6,
        CH7,
        CH8,
        CH9,
        CH10,
        CH11,
        CH12,
        CH13,
        CH14,
        CH15,
        CH_NUM
    };

    /**
     * @brief Conversion sample time. See reference manual.
     */
    enum SampleTime : uint8_t
    {
        CYCLES_3   = 0x0,
        CYCLES_15  = 0x1,
        CYCLES_28  = 0x2,
        CYCLES_56  = 0x3,
        CYCLES_84  = 0x4,
        CYCLES_112 = 0x5,
        CYCLES_144 = 0x6,
        CYCLES_480 = 0x7
    };

    explicit InternalADC(ADC_TypeDef& ADCx_, const float V_SUPPLY_ = 5.0,
                         const bool isUsingDMA_    = false,
                         DMA_Stream_TypeDef* DMAx_ = DMA1_Stream0);

    ~InternalADC();

    /**
     * @brief ADC Initialization
     *
     * The ADC clock must be set before hand as well as GPIO configuration
     * and DMA if used. Also the clock for the analog circuitry should be set
     * accordingly to the device datasheet.
     */
    bool init() override;

    bool enableChannel(Channel channel);

    bool enableChannel(Channel channel, SampleTime sampleTime);

    ADCData getVoltage(Channel channel);

    bool selfTest() override;

    ADCData sampleImpl() override;

private:
    inline void resetRegisters();

    inline void startInjectedConversion();

    inline void startRegularConversion();

    inline bool addInjectedChannel(Channel channel);

    inline bool addRegularChannel(Channel channel);

    inline void setChannelSampleTime(Channel channel, SampleTime sampleTime);

    inline void enableADCClock();

    inline void disableADCClock();

    ADC_TypeDef& ADCx;
    const float V_SUPPLY = 5.0;

    uint8_t activeChannels = 0;
    uint64_t timestamp;

    // Raw value used by DMA
    uint16_t values[CH_NUM] = {};

    // Maps the channel number with the index in the ADC's regular sequence
    int8_t indexMap[CH_NUM];

    /**
     * @brief Determines whether to use regular channels or injected channels
     *
     * We'll use up to 4 injected channel by dafult and up to 16 channels when
     * using DMA.
     *
     * The differantiation is necessary because whitout DMA it is much more
     * simplier to use injected channel for multichannel readings. Otherwise we
     * would need to handle each channel's end of conversion interrupt or go
     * through
     */
    const bool isUsingDMA;
    DMA_Stream_TypeDef* DMAx_Streamx;
    DMA_TypeDef* DMAx = DMA1;
    uint8_t streamNum = 0;
    uint32_t transferCompleteMask;
    uint32_t transferErrorMask;
    volatile uint32_t* statusReg;
    volatile uint32_t* clearFlagReg;

    static constexpr int INJECTED_CHANNEL_N = 4;
    static constexpr int RESOLUTION         = 4096;  ///< 12 bits
};

}  // namespace Boardcore
