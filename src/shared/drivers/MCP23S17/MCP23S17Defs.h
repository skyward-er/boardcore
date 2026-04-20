/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Tommaso Lamon
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

#include <cstdint>

namespace Boardcore
{
namespace MCP23S17Defs
{

static constexpr uint16_t WRITE_OPCODE = 0b01000000;
static constexpr uint16_t READ_OPCODE  = 0b01000001;

enum PORT
{
    PORT_A = 0,
    PORT_B = 1
};

enum PIN
{
    PIN0 = 0,
    PIN1 = 1,
    PIN2 = 2,
    PIN3 = 3,
    PIN4 = 4,
    PIN5 = 5,
    PIN6 = 6,
    PIN7 = 7
};

enum class MODE
{
    INPUT         = 0,
    OUTPUT        = 1,
    INPUT_PULL_UP = 2,
};

enum CONFIG_FIELDS
{
    BANK   = 0x07,
    MIRROR = 0x06,
    SEQOP  = 0x05,
    DISSLW = 0x04,
    HAEN   = 0x03,
    ODR    = 0x02,
    INTPOL = 0x01
};

enum GPIO_REG
{
    IODIR_BASE    = 0x00,
    IPOL_BASE     = 0x02,
    GPINTEN_BASE  = 0x04,
    GPPU_BASE     = 0x0C,
    GPIO_EXT_BASE = 0x12,  // Nel datasheet è GPIO
    OLAT_BASE     = 0x14,
};

enum CTRL_REG
{
    DEFVAL_BASE = 0x06,
    INTCON_BASE = 0x08,
    IOCON_BASE  = 0x0A,
    INTF_BASE   = 0x0E,
    INTCAP_BASE = 0x10
};

}  // namespace MCP23S17Defs
}  // namespace Boardcore
