/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include "SX1278Common.h"

namespace Boardcore
{

class EbyteFrontend : public SX1278::ISX1278Frontend
{
public:
    EbyteFrontend(miosix::GpioPin tx_enable, miosix::GpioPin rx_enable)
        : tx_enable(tx_enable), rx_enable(rx_enable)
    {
    }

    bool isOnPaBoost() override { return true; }
    int maxInPower() override
    {
        return 15;
    }

    void enableRx() override { rx_enable.high(); }
    void disableRx() override { rx_enable.low(); }
    void enableTx() override { tx_enable.high(); }
    void disableTx() override { tx_enable.low(); }

private:
    miosix::GpioPin tx_enable;
    miosix::GpioPin rx_enable;
};

class RA01Frontend : public SX1278::ISX1278Frontend
{
public:
    RA01Frontend() {}

    bool isOnPaBoost() override { return true; }
    int maxInPower() override { return 17; }

    void enableRx() override {}
    void disableRx() override {}
    void enableTx() override {}
    void disableTx() override {}
};

class Skyward433Frontend : public SX1278::ISX1278Frontend
{
public:
    Skyward433Frontend() {}

    bool isOnPaBoost() override { return false; }
    int maxInPower() override { return 15; }

    void enableRx() override {}
    void disableRx() override {}
    void enableTx() override {}
    void disableTx() override {}
};

}  // namespace Boardcore