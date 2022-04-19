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

#include "HX711.h"

#include <drivers/timer/TimestampTimer.h>
#include <interfaces-impl/gpio_impl.h>
#include <miosix.h>

namespace Boardcore
{

HX711::HX711(SPIBusInterface &bus, miosix::GpioPin sckPin, SPIBusConfig config,
             unsigned char sckAlternateFunction)
    : bus(bus), sckPin(sckPin), config(config),
      sckAlternateFunction(sckAlternateFunction)
{
}

SPIBusConfig HX711::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_128;
    spiConfig.mode         = SPI::Mode::MODE_1;
    return spiConfig;
}

bool HX711::init() { return true; }

bool HX711::selfTest() { return true; }

HX711Data HX711::sampleImpl()
{
    int32_t sample;

    bus.configure(config);
    sample = bus.read16() << 8;
    sample |= bus.read();
    sckPin.mode(miosix::Mode::OUTPUT);
    sckPin.high();
    miosix::delayUs(1);
    sckPin.low();
    sckPin.mode(miosix::Mode::ALTERNATE);
    sckPin.alternateFunction(sckAlternateFunction);

    if (sample & static_cast<int32_t>(0x800000))
        sample |= static_cast<uint32_t>(0xFF) << 24;

    if (sample == static_cast<int32_t>(0xFFFFFFFF))
        return lastSample;

    return {TimestampTimer::getInstance().getTimestamp(),
            static_cast<float>(sample + offset) / scale};
}

void HX711::setScale(float scale) { this->scale = scale; }

void HX711::setZero() { offset = -lastSample.load * scale; }

void HX711::setZero(float offset) { this->offset = offset * scale; }

}  // namespace Boardcore
