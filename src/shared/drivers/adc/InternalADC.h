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
 * The driver uses basic ADC features, that is the single conversion mode.
 * A previous version of the driver featured injected and regular channels with
 * also DMA. Since ADC conversion time is very low, the driver has been
 * simplified to provide better usability and simpler implementation.
 *
 * @warning This driver has been tested on f205, f407, f429, f767 and f769
 */
class InternalADC : public Sensor<InternalADCData>
{
public:
    /**
     * @brief ADC channels enumeration.
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
        CH16,
        CH17,
        CH18,
        CH_NUM
    };

    /**
     * @brief Conversion sample time. See reference manual.
     *
     * CYCLES_3 is not exposed because in 12-bit mode the minimum is 15
     */
    enum SampleTime : uint8_t
    {
        // CYCLES_3   = 0x0,
        CYCLES_15  = 0x1,
        CYCLES_28  = 0x2,
        CYCLES_56  = 0x3,
        CYCLES_84  = 0x4,
        CYCLES_112 = 0x5,
        CYCLES_144 = 0x6,
        CYCLES_480 = 0x7
    };

    /**
     * @brief Resets the ADC configuration and automatically enables the
     * peripheral clock.
     */
    explicit InternalADC(ADC_TypeDef* adc);

    ~InternalADC();

    /**
     * @brief ADC Initialization.
     *
     * The ADC clock must be set beforehand as well as GPIO configuration. Also
     * the clock for the analog circuitry should be set accordingly to the
     * device datasheet.
     */
    bool init() override;

    bool selfTest() override;

    InternalADCData sampleImpl() override;

    void enableChannel(Channel channel, SampleTime sampleTime = CYCLES_480);

    void disableChannel(Channel channel);

    void enableTemperature(SampleTime sampleTime = CYCLES_480);

    void disableTemperature();

    void enableVbat(SampleTime sampleTime = CYCLES_480);

    void disableVbat();

    ADCData getVoltage(Channel channel);

    TemperatureData getTemperature();

    ADCData getVbatVoltage();

private:
    void resetRegisters();

    void setChannelSampleTime(Channel channel, SampleTime sampleTime);

    uint16_t readChannel(Channel channel);

    ADC_TypeDef* adc;

    bool channelsEnabled[CH_NUM];
    bool tempEnabled = false;
    bool vbatEnabled = false;
};

}  // namespace Boardcore
