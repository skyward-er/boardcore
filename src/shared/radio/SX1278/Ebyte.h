/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "SX1278Fsk.h"
#include "SX1278Lora.h"

namespace Boardcore
{

/**
 * @brief Specialized SX1278 driver for the E32 400M30S module
 */
class EbyteFsk : public SX1278Fsk
{
public:
    EbyteFsk(SPIBus &bus, miosix::GpioPin cs, SPI::ClockDivider clock_divider,
             miosix::GpioPin tx_enable, miosix::GpioPin rx_enable)
        : SX1278Fsk(bus, cs, clock_divider), tx_enable(tx_enable),
          rx_enable(rx_enable)
    {
        // Make sure both the frontends are disabled!
        tx_enable.low();
        rx_enable.low();
    }

    [[nodiscard]] Error configure(const Config &config) override;

private:
    void enableRxFrontend() override;
    void disableRxFrontend() override;
    void enableTxFrontend() override;
    void disableTxFrontend() override;

    miosix::GpioPin tx_enable;
    miosix::GpioPin rx_enable;
};

/**
 * @brief Specialized SX1278 driver for the E32 400M30S module
 */
class EbyteLora : public SX1278Lora
{
public:
    EbyteLora(SPIBus &bus, miosix::GpioPin cs, SPI::ClockDivider clock_divider,
              miosix::GpioPin tx_enable, miosix::GpioPin rx_enable)
        : SX1278Lora(bus, cs, clock_divider), tx_enable(tx_enable),
          rx_enable(rx_enable)
    {
        // Make sure both the frontends are disabled!
        tx_enable.low();
        rx_enable.low();
    }

    [[nodiscard]] Error configure(const Config &config) override;

private:
    void enableRxFrontend() override;
    void disableRxFrontend() override;
    void enableTxFrontend() override;
    void disableTxFrontend() override;

    miosix::GpioPin tx_enable;
    miosix::GpioPin rx_enable;
};

}  // namespace Boardcore
