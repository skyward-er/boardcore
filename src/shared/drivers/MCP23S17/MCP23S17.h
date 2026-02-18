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

#include <drivers/spi/SPIDriver.h>
#include <drivers/MCP23S17/MCP23S17Defs.h>

namespace Boardcore
{

class MCP23S17
{
public:
    MCP23S17(SPIBusInterface& bus, miosix::GpioPin cs,
             SPIBusConfig config = getDefaultSPIConfig())
        : spiSlave(bus, cs, config) {};

    static SPIBusConfig getDefaultSPIConfig();

    void init();

    uint8_t getGpioAddress(MCP23S17Defs::GPIO_REG reg);
    uint8_t getCtrlAddress(MCP23S17Defs::CTRL_REG reg);

    // Helper functions
    void wipeRegister(uint8_t address);

    uint8_t readRegister(uint8_t address);
    bool readBit(uint8_t regAddress, uint8_t bitNumber);

    void writeBit(uint8_t regAddress, uint8_t bitNumber, bool value);
    void writeRegister(uint8_t regAddress, uint8_t value);

    // Pin I/O Config
    void setPinIn_A(uint8_t pinNumber);
    void setPinIn_B(uint8_t pinNumber);

    void setPinOut_A(uint8_t pinNumber);
    void setPinOut_B(uint8_t pinNumber);

    // Pin Polarity Config
    void setPinPolarity_A(uint8_t pinNumber, bool polarity);
    void setPinPolarity_B(uint8_t pinNumber, bool polarity);

    // General IC Control Register Config
    void setBANK(bool value);

    void setMIRROR(bool value);

    void setSEQOP(bool value);

    void setDISSLW(bool value);

    void setHAEN(bool value);

    void setODR(bool value);

    void setINTPOL(bool value);

    // Interrupt On Change Config
    void enableInterruptOnChange_A(uint8_t pinNumber);
    void enableInterruptOnChange_B(uint8_t pinNumber);

    void disableInterruptOnChange_A(uint8_t pinNumber);
    void disableInterruptOnChange_B(uint8_t pinNumber);

    // Pull-up Config
    void enablePullUp_A(uint8_t pinNumber);
    void enablePullUp_B(uint8_t pinNumber);

    void disablePullUp_A(uint8_t pinNumber);
    void disablePullUp_B(uint8_t pinNumber);

    void setDefaultValue_A(uint8_t pinNumber, bool value);
    void setDefaultValue_B(uint8_t pinNumber, bool value);

    void setInterruptComparison_A(uint8_t pinNumber, bool mode);
    void setInterruptComparison_B(uint8_t pinNumber, bool mode);

    uint8_t readInterruptFlag_A();
    uint8_t readInterruptFlag_B();

    uint8_t readInterruptCapture_A();
    uint8_t readInterruptCapture_B();

    bool readPin_A(uint8_t pinNumber);
    bool readPin_B(uint8_t pinNumber);

    bool readLatch_A(uint8_t pinNumber);
    bool readLatch_B(uint8_t pinNumber);

    void writePin_A(uint8_t pinNumber, bool value);
    void writePin_B(uint8_t pinNumber, bool value);

    void writeLatch_A(uint8_t pinNumber, bool value);
    void writeLatch_B(uint8_t pinNumber, bool value);

private:
    SPISlave spiSlave;
    MCP23S17Defs::Bank activeBank = MCP23S17Defs::Bank::Bank0;
};

}  // namespace Boardcore
