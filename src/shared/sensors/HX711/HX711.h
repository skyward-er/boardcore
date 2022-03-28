/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "HX711Data.h"

namespace Boardcore
{

/**
 * @brief Load cell transducer.
 *
 * HX711 is a precision 24-bit analog-to-digital converter (ADC) designed for
 * weigh scales and industrial control applications to interface directly with a
 * bridge sensor.
 *
 * The input multiplexer selects either Channel A or B differential input to the
 * low-noise programmable gain amplifier (PGA). Channel A can be programmed with
 * a gain of 128 or 64, corresponding to a full-scale differential input voltage
 * of ±20mV or ±40mV respectively, when a 5V supply is connected to AVDD analog
 * power supply pin. Channel B has a fixed gain of 32. On-chip power supply
 * regulator eliminates the need for an external supply regulator to provide
 * analog power for the ADC and the sensor. Clock input is flexible. It can be
 * from an external clock source, a crystal, or the on-chip oscillator that does
 * not require any external component. On-chip power-on-reset circuitry
 * simplifies digital interface initialization. There is no programming needed
 * for the internal registers. All controls to the HX711 are through the pins.
 *
 * Reference: https://github.com/bogde/HX711
 */
class HX711 : public Sensor<HX711Data>
{
public:
    HX711(SPIBusInterface& bus, miosix::GpioPin sckPin,
          SPIBusConfig config                = getDefaultSPIConfig(),
          unsigned char sckAlternateFunction = 5);

    static SPIBusConfig getDefaultSPIConfig();

    bool init() override;

    bool selfTest() override;

    /**
     * @brief Sets the scale value.
     *
     * This value is used to convert the raw data into weight.
     */
    void setScale(float scale);

    /**
     * @brief Sets the offset to zero the current measurement.
     */
    void setZero();

    /**
     * @brief Sets the offset to the given value.
     *
     * @param offset Offset in Kg.
     */
    void setZero(float offset);

private:
    HX711Data sampleImpl() override;

    SPIBusInterface& bus;
    miosix::GpioPin sckPin;
    const SPIBusConfig config;
    unsigned char sckAlternateFunction;

    float scale    = 1;
    int32_t offset = 0;
};

}  // namespace Boardcore
