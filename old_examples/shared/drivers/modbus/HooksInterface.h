/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

/* Class that provides an interface made by a set of function interface used by
 * Modbus slave engine to handle the server requests.
 * By default the functions will return the error code "function unsupported",
 * user must provide a correct implementation by subclassing this class
 */

#pragma once

#include "Common.h"
#include "ExceptionCodes.h"

class HooksInterface
{
public:
    HooksInterface() {}

    ~HooksInterface() {}

    virtual uint8_t ReadCoil(bool& value, uint16_t addr)
    {
        return modbus::EXC_ILLEGAL_FUN;
    }

    virtual uint8_t WriteCoil(bool value, uint16_t addr)
    {
        return modbus::EXC_ILLEGAL_FUN;
    }

    virtual uint8_t ReadRegister(uint16_t& value, uint16_t addr)
    {
        return modbus::EXC_ILLEGAL_FUN;
    }

    virtual uint8_t WriteRegister(uint16_t value, uint16_t addr)
    {
        return modbus::EXC_ILLEGAL_FUN;
    }

private:
    HooksInterface(const HooksInterface& other) = delete;
    HooksInterface& operator=(const HooksInterface& other) = delete;
    bool operator==(const HooksInterface& other)           = delete;
};
