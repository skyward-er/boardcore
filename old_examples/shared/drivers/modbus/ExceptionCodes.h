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

#pragma once

#include "Common.h"

namespace modbus
{
static constexpr uint8_t EXC_ILLEGAL_FUN   = 0x01;  ///< Illegal function
static constexpr uint8_t EXC_ILLEGAL_ADDR  = 0x02;  ///< Illegal data address
static constexpr uint8_t EXC_ILLEGAL_VAL   = 0x03;  ///< Illegal data value
static constexpr uint8_t EXC_SLAVE_FAILURE = 0x04;  ///< Slave internal failure
static constexpr uint8_t ACKNOWLEDGE       = 0x05;  ///< Acknowledge
static constexpr uint8_t SLAVE_BUSY        = 0x06;  ///< Slave busy
static constexpr uint8_t EXC_MEM_PARITY    = 0x08;  ///< Memory parity error
}  // namespace modbus
