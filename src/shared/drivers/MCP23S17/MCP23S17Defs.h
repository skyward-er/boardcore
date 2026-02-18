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

namespace Boardcore
{
namespace MCP23S17Defs
{

static constexpr uint16_t WRITE_OPCODE = 0b01000000;
static constexpr uint16_t READ_OPCODE  = 0b01000001;

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

enum class Bank
{
    Bank0,
    Bank1,
    Last
};

enum class GPIO_REG
{
    IODIRA,
    IODIRB,
    IOPOLA,
    IOPOLB,
    GPINTENA,
    GPINTENB,
    GPPUA,
    GPPUB,
    GPIOA_EXT,  // Aggiunto _EXT causa conflitto nomi con miosix -> su
                // datasheet é solo GPIOA(B)
    GPIOB_EXT,
    OLATA,
    OLATB,
    Last
};

inline constexpr uint8_t GPIO_REG_LUT[(int)Bank::Last][(int)GPIO_REG::Last] = {
    {
        // BANK 0
        0x00,  // IODIRA
        0x01,  // IODIRB
        0x02,  // IOPOLA
        0x03,  // IOPOLB
        0x04,  // GPINTENA
        0x05,  // GPINTENB
        0x0C,  // GPPUA
        0x0D,  // GPPUB
        0x12,  // GPIOA_EXT
        0x13,  // GPIOB_EXT
        0x14,  // OLATA
        0x15   // OLATB
    },
    {
        // BANK 1
        0x00,  // IODIRA
        0x01,  // IOPOLA
        0x02,  // GPINTENA
        0x06,  // GPPUA
        0x09,  // GPIOA_EXT
        0x0A,  // OLATA
        0x10,  // IODIRB
        0x11,  // IOPOLB
        0x12,  // GPINTENB
        0x16,  // GPIOB_EXT
        0x19,  // GPPUB
        0x1A   // OLATB
    }};

enum class CTRL_REG
{
    DEFVALA,
    INTCONA,
    IOCON,
    INTFA,
    INTCAPA,
    DEFVALB,
    INTCONB,
    INTFB,
    INTCAPB,
    Last
};

inline constexpr uint8_t CTRL_REG_LUT[(int)Bank::Last][(int)CTRL_REG::Last] = {
    {
        // BANK 0
        0x06,  // DEFVALA
        0x08,  // INTCONA
        0x0A,  // IOCON
        0x0E,  // INTFA
        0x10,  // INTCAPA
        0x07,  // DEFVALB
        0x09,  // INTCONB
        0x0F,  // INTFB
        0x11   // INTCAPB
    },
    {
        // BANK 1
        0x03,  // DEFVALA
        0x04,  // INTCONA
        0x05,  // IOCON
        0x07,  // INTFA
        0x08,  // INTCAPA
        0x13,  // DEFVALB
        0x14,  // INTCONB
        0x17,  // INTFB
        0x18   // INTCAPB
    }

};

}  // namespace MCP23S17Defs
}  // namespace Boardcore
