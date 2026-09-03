/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Tommaso Lamon, Pietro Bortolus
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

#include "MCP23S17.h"

namespace Boardcore
{

using namespace MCP23S17Defs;

SPIBusConfig MCP23S17::getDefaultSPIConfig()
{
    SPIBusConfig config{};

    config.clockDivider = SPI::ClockDivider::DIV_32;
    config.mode         = SPI::Mode::MODE_0;

    return config;
}

uint8_t MCP23S17::readRegister(uint8_t address)
{
    uint8_t buf[3];

    buf[0] = MCP23S17Defs::READ_OPCODE;
    buf[1] = address;
    buf[2] = 0;

    {
        miosix::Lock<miosix::FastMutex> lock(spiMutex);
        SPITransaction spiTransaction{this->spiSlave};
        spiTransaction.transfer(buf, sizeof(buf));
    }

    return buf[2];
}

bool MCP23S17::readBit(uint8_t address, uint8_t pinNumber)
{
    return (readRegister(address) >> pinNumber) & 1;
}

void MCP23S17::writeBit(uint8_t address, uint8_t bitNumber, bool value)
{
    uint8_t old  = readRegister(address);
    uint8_t mask = (1 << bitNumber);
    uint8_t newValue;

    if (value)
        newValue = old | mask;
    else
        newValue = old & ~mask;

    uint32_t msg =
        (MCP23S17Defs::WRITE_OPCODE << 16) | (address << 8) | newValue;

    {
        miosix::Lock<miosix::FastMutex> lock(spiMutex);
        SPITransaction spiTransaction{this->spiSlave};
        spiTransaction.write24(msg);
    }
}

void MCP23S17::writeRegister(uint8_t address, uint8_t value)
{
    uint32_t data = (MCP23S17Defs::WRITE_OPCODE << 16) | (address << 8) | value;

    {
        miosix::Lock<miosix::FastMutex> lock(spiMutex);
        SPITransaction spiTransaction{this->spiSlave};
        spiTransaction.write24(data);
    }
}

void MCP23S17::init()
{
    // Setup IOCON
    setHAEN(false);
    setMIRROR(false);
    setSEQOP(true);
    setDISSLW(true);
    setINTPOL(false);

    // Setup pins (all inputs)
    writeRegister(GPIO_REG::IODIR_BASE + PORT::PORT_A, 0xFF);
    writeRegister(GPIO_REG::IODIR_BASE + PORT::PORT_B, 0xFF);

    // Setup Pull-Ups (default off)
    writeRegister(GPIO_REG::GPPU_BASE + PORT::PORT_A, 0x00);
    writeRegister(GPIO_REG::GPPU_BASE + PORT::PORT_B, 0x00);

    // Clear Interrupt configs
    writeRegister(GPIO_REG::GPINTEN_BASE + PORT::PORT_A, 0x00);
    writeRegister(GPIO_REG::GPINTEN_BASE + PORT::PORT_B, 0x00);
    writeRegister(CTRL_REG::DEFVAL_BASE + PORT::PORT_A, 0x00);
    writeRegister(CTRL_REG::DEFVAL_BASE + PORT::PORT_B, 0x00);
    writeRegister(CTRL_REG::INTCON_BASE + PORT::PORT_A, 0x00);
    writeRegister(CTRL_REG::INTCON_BASE + PORT::PORT_B, 0x00);
}

/**
 * Expander configuration methods
 */

void MCP23S17::setMIRROR(bool value)
{
    writeBit(CTRL_REG::IOCON_BASE, CONFIG_FIELDS::MIRROR, value);
}

void MCP23S17::setSEQOP(bool value)
{
    writeBit(CTRL_REG::IOCON_BASE, CONFIG_FIELDS::SEQOP, value);
}

void MCP23S17::setDISSLW(bool value)
{
    writeBit(CTRL_REG::IOCON_BASE, CONFIG_FIELDS::DISSLW, value);
}

void MCP23S17::setHAEN(bool value)
{
    writeBit(CTRL_REG::IOCON_BASE, CONFIG_FIELDS::HAEN, value);
}

void MCP23S17::setODR(bool value)
{
    writeBit(CTRL_REG::IOCON_BASE, CONFIG_FIELDS::ODR, value);
}

void MCP23S17::setINTPOL(bool value)
{
    writeBit(CTRL_REG::IOCON_BASE, CONFIG_FIELDS::INTPOL, value);
}

/**
 * Pin specific methods
 */

void MCP23S17::setPinMode(PORT port, PIN pinNumber, MODE mode)
{
    writeBit(GPIO_REG::IODIR_BASE + port, pinNumber,
             (mode == MODE::INPUT) || (mode == MODE::INPUT_PULL_UP));

    writeBit(GPIO_REG::GPPU_BASE + port, pinNumber,
             (mode == MODE::INPUT_PULL_UP));
}

void MCP23S17::setPinPolarity(PORT port, PIN pinNumber, bool polarity)
{
    writeBit(GPIO_REG::IPOL_BASE + port, pinNumber, polarity);
}

void MCP23S17::setDefaultValue(PORT port, PIN pinNumber, bool value)
{
    writeBit(CTRL_REG::DEFVAL_BASE + port, pinNumber, value);
}

void MCP23S17::enableInterruptOnChange(PORT port, PIN pinNumber)
{
    writeBit(GPIO_REG::GPINTEN_BASE + port, pinNumber, 1);
}

void MCP23S17::disableInterruptOnChange(PORT port, PIN pinNumber)
{
    writeBit(GPIO_REG::GPINTEN_BASE + port, pinNumber, 0);
}

void MCP23S17::setInterruptComparison(PORT port, PIN pinNumber, bool mode)
{
    writeBit(CTRL_REG::INTCON_BASE + port, pinNumber, mode);
}

uint8_t MCP23S17::readInterruptFlag(PORT port)
{
    return readRegister(CTRL_REG::INTF_BASE + port);
}

uint8_t MCP23S17::readInterruptCapture(PORT port)
{
    return readRegister(CTRL_REG::INTCAP_BASE + port);
}

bool MCP23S17::getPinValue(PORT port, PIN pinNumber)
{
    return readBit(GPIO_REG::GPIO_EXT_BASE + port, pinNumber);
}

void MCP23S17::setPinValue(PORT port, PIN pinNumber, bool value)
{
    writeBit(GPIO_REG::GPIO_EXT_BASE + port, pinNumber, value);
}

}  // namespace Boardcore
