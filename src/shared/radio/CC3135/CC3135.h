/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <memory>

#include "CC3135Defs.h"
#include "CC3135Iface.h"

/*
TODO(davide.mor): Write a small description of the CC3135
*/

namespace Boardcore
{

/**
 * @brief Abstraction over the comunication protocol.
 */
class CC3135Proto
{
public:
    explicit CC3135Proto(std::unique_ptr<ICC3135Iface> &&iface)
        : iface(std::move(iface)), tx_seq_num(0)
    {
    }

    void handleIntr();

    void readPacket(uint8_t *buf);
    void writePacket(uint8_t *buf, size_t size);

private:
    std::unique_ptr<ICC3135Iface> iface;
    uint8_t tx_seq_num;
};

class CC3135
{
public:
    explicit CC3135(std::unique_ptr<ICC3135Iface> &&iface)
        : proto(std::move(iface))
    {
    }

    void handleIntr() { proto.handleIntr(); }

    //! Dummy packet read, used to test comunication.
    void dummyRead();

    //! Retrieve information about the device
    CC3135Defs::DeviceVersion getVersion();

private:
    CC3135Proto proto;
};

}  // namespace Boardcore
