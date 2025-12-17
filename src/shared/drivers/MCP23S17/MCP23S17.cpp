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

#include "MCP23S17.h"

#include "MCP23S17Defs.h"

namespace Boardcore
{

using namespace MCP23S17Defs;

SPIBusConfig MCP23S17::getDefaultSPIConfig()
{
    SPIBusConfig config{};

    config.clockDivider = SPI::ClockDivider::DIV_32;
    config.mode         = SPI::Mode::MODE_2;

    return config;
}

uint8_t MCP23S17::getGpioAddress(GPIO_REG reg)
{
    return GPIO_REG_LUT[(int)activeBank][(int)reg];
}

uint8_t MCP23S17::getCtrlAddress(CTRL_REG reg)
{
    return CTRL_REG_LUT[(int)activeBank][(int)reg];
}

void MCP23S17::wipeRegister(uint8_t address)
{
    uint32_t data;
    SPITransaction spiTransaction{this->spiSlave};

    data = (MCP23S17Defs::WRITE_OPCODE << 16) | (address << 8) | 0x00;

    spiTransaction.write24(data);
}

uint8_t MCP23S17::readRegister(uint8_t address)
{
    uint32_t msg;
    uint8_t result;

    SPITransaction spiTransaction{this->spiSlave};

    msg = (MCP23S17Defs::READ_OPCODE << 8) | (address);

    spiSlave.bus.select(spiSlave.cs);
    spiSlave.bus.write16(msg);
    result = spiSlave.bus.read();
    spiSlave.bus.deselect(spiSlave.cs);

    return result;
}

bool MCP23S17::readBit(uint8_t regAddress, uint8_t pinNumber)
{
    return (readRegister(regAddress) >> pinNumber) & 1;
}

void MCP23S17::writeBit(uint8_t regAddress, uint8_t bitNumber, bool value)
{
    uint8_t old  = readRegister(regAddress);
    uint8_t mask = (1 << bitNumber);
    uint8_t newValue;

    if (value)
        newValue = old | mask;
    else
        newValue = old & ~mask;

    uint32_t msg =
        (MCP23S17Defs::WRITE_OPCODE << 16) | (regAddress << 8) | newValue;

    SPITransaction spiTransaction{this->spiSlave};

    spiTransaction.write24(msg);
}

void MCP23S17::writeRegister(uint8_t regAddress, uint8_t value)
{
    uint32_t data;
    SPITransaction spiTransaction{this->spiSlave};

    data = (MCP23S17Defs::WRITE_OPCODE << 16) | (regAddress << 8) | value;
    spiTransaction.write24(data);
}

void MCP23S17::init()
{
    bool enableHAEN = false;
    bool mirror     = false;
    bool seqOP      = true;
    bool disableSLW = true;
    bool intPOL     = false;

    // Setup IOCON

    setHAEN(enableHAEN);
    setMIRROR(mirror);
    setSEQOP(seqOP);
    setDISSLW(disableSLW);
    setINTPOL(intPOL);

    // Setup pins (all inputs)
    writeRegister(getGpioAddress(GPIO_REG::IODIRA), 0xFF);
    writeRegister(getGpioAddress(GPIO_REG::IODIRB), 0xFF);

    // Setup Pull-Ups (default off)
    writeRegister(getGpioAddress(GPIO_REG::GPPUA), 0x00);
    writeRegister(getGpioAddress(GPIO_REG::GPPUB), 0x00);

    // Cleared Interrupt configs
    writeRegister(getGpioAddress(GPIO_REG::GPINTENA), 0x00);
    writeRegister(getGpioAddress(GPIO_REG::GPINTENB), 0x00);
    writeRegister(getCtrlAddress(CTRL_REG::DEFVALA), 0x00);
    writeRegister(getCtrlAddress(CTRL_REG::DEFVALB), 0x00);
    writeRegister(getCtrlAddress(CTRL_REG::INTCONA), 0x00);
    writeRegister(getCtrlAddress(CTRL_REG::INTCONB), 0x00);
}

void MCP23S17::setPinIn_A(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::IODIRA), pinNumber, 1);
}

void MCP23S17::setPinIn_B(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::IODIRB), pinNumber, 1);
}

void MCP23S17::setPinOut_A(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::IODIRA), pinNumber, 0);
}

void MCP23S17::setPinOut_B(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::IODIRB), pinNumber, 0);
}

void MCP23S17::setPinPolarity_A(uint8_t pinNumber, bool polarity)
{
    writeBit(getGpioAddress(GPIO_REG::IOPOLA), pinNumber, polarity);
}

void MCP23S17::setPinPolarity_B(uint8_t pinNumber, bool polarity)
{
    writeBit(getGpioAddress(GPIO_REG::IOPOLB), pinNumber, polarity);
}

void MCP23S17::setBANK(bool value)
{
    writeBit(getCtrlAddress(CTRL_REG::IOCON), CONFIG_FIELDS::BANK, value);

    activeBank = value ? Bank::Bank1 : Bank::Bank0;
}

void MCP23S17::setMIRROR(bool value)
{
    writeBit(getCtrlAddress(CTRL_REG::IOCON), CONFIG_FIELDS::MIRROR, value);
}

void MCP23S17::setSEQOP(bool value)
{
    writeBit(getCtrlAddress(CTRL_REG::IOCON), CONFIG_FIELDS::SEQOP, value);
}

void MCP23S17::setDISSLW(bool value)
{
    writeBit(getCtrlAddress(CTRL_REG::IOCON), CONFIG_FIELDS::DISSLW, value);
}

void MCP23S17::setHAEN(bool value)
{
    writeBit(getCtrlAddress(CTRL_REG::IOCON), CONFIG_FIELDS::HAEN, value);
}

void MCP23S17::setODR(bool value)
{
    writeBit(getCtrlAddress(CTRL_REG::IOCON), CONFIG_FIELDS::ODR, value);
}

void MCP23S17::setINTPOL(bool value)
{
    writeBit(getCtrlAddress(CTRL_REG::IOCON), CONFIG_FIELDS::INTPOL, value);
}

void MCP23S17::enableInterruptOnChange_A(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::GPINTENA), pinNumber, 1);
}

void MCP23S17::enableInterruptOnChange_B(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::GPINTENB), pinNumber, 1);
}

void MCP23S17::disableInterruptOnChange_A(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::GPINTENA), pinNumber, 0);
}

void MCP23S17::disableInterruptOnChange_B(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::GPINTENB), pinNumber, 0);
}

void MCP23S17::enablePullUp_A(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::GPPUA), pinNumber, 1);
}

void MCP23S17::enablePullUp_B(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::GPPUB), pinNumber, 1);
}

void MCP23S17::disablePullUp_A(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::GPPUA), pinNumber, 0);
}

void MCP23S17::disablePullUp_B(uint8_t pinNumber)
{
    writeBit(getGpioAddress(GPIO_REG::GPPUB), pinNumber, 0);
}

void MCP23S17::setDefaultValue_A(uint8_t pinNumber, bool value)
{
    writeBit(getCtrlAddress(CTRL_REG::DEFVALA), pinNumber, value);
}

void MCP23S17::setDefaultValue_B(uint8_t pinNumber, bool value)
{
    writeBit(getCtrlAddress(CTRL_REG::DEFVALB), pinNumber, value);
}

void MCP23S17::setInterruptComparison_A(uint8_t pinNumber, bool mode)
{
    writeBit(getCtrlAddress(CTRL_REG::INTCONA), pinNumber, mode);
}

void MCP23S17::setInterruptComparison_B(uint8_t pinNumber, bool mode)
{
    writeBit(getCtrlAddress(CTRL_REG::INTCONB), pinNumber, mode);
}

uint8_t MCP23S17::readInterruptFlag_A()
{
    uint8_t result;

    result = readRegister(getCtrlAddress(CTRL_REG::INTFA));

    return result;
}

uint8_t MCP23S17::readInterruptFlag_B()
{
    uint8_t result;

    result = readRegister(getCtrlAddress(CTRL_REG::INTFB));

    return result;
}

uint8_t MCP23S17::readInterruptCapture_A()
{
    uint8_t result;

    result = readRegister(getCtrlAddress(CTRL_REG::INTCAPA));

    return result;
}

uint8_t MCP23S17::readInterruptCapture_B()
{
    uint8_t result;

    result = readRegister(getCtrlAddress(CTRL_REG::INTCAPB));

    return result;
}

bool MCP23S17::readPin_A(uint8_t pinNumber)
{
    bool result;

    result = readBit(getGpioAddress(GPIO_REG::GPIOA_EXT), pinNumber);

    return result;
}

bool MCP23S17::readPin_B(uint8_t pinNumber)
{
    bool result;

    result = readBit(getGpioAddress(GPIO_REG::GPIOB_EXT), pinNumber);

    return result;
}

bool MCP23S17::readLatch_A(uint8_t pinNumber)
{
    bool result;

    result = readBit(getGpioAddress(GPIO_REG::OLATA), pinNumber);

    return result;
}

bool MCP23S17::readLatch_B(uint8_t pinNumber)
{
    bool result;

    result = readBit(getGpioAddress(GPIO_REG::OLATB), pinNumber);

    return result;
}

void MCP23S17::writePin_A(uint8_t pinNumber, bool value)
{
    writeBit(getGpioAddress(GPIO_REG::GPIOA_EXT), pinNumber, value);
}

void MCP23S17::writePin_B(uint8_t pinNumber, bool value)
{
    writeBit(getGpioAddress(GPIO_REG::GPIOB_EXT), pinNumber, value);
}

void MCP23S17::writeLatch_A(uint8_t pinNumber, bool value)
{
    writeBit(getGpioAddress(GPIO_REG::OLATA), pinNumber, value);
}

void MCP23S17::writeLatch_B(uint8_t pinNumber, bool value)
{
    writeBit(getGpioAddress(GPIO_REG::OLATB), pinNumber, value);
}

}  // namespace Boardcore
