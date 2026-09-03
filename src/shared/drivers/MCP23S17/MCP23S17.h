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

#pragma once

#include <drivers/spi/SPIDriver.h>
#include <miosix.h>

#include "MCP23S17Defs.h"

namespace Boardcore
{

struct ExternalGpioPin
{
    constexpr ExternalGpioPin(MCP23S17Defs::PORT port, MCP23S17Defs::PIN pin)
        : port(port), pin(pin)
    {
    }

    MCP23S17Defs::PORT port;
    MCP23S17Defs::PIN pin;

    constexpr MCP23S17Defs::PORT getPort() const { return port; }
    constexpr MCP23S17Defs::PIN getPin() const { return pin; }
};

class MCP23S17
{
public:
    /**
     * @brief Class constructor.
     * @param bus SPI Bus
     * @param cs Chip Select Pin
     * @param config SPI Bus Configuration
     */
    MCP23S17(SPIBusInterface& bus, miosix::GpioPin cs,
             SPIBusConfig config = getDefaultSPIConfig())
        : spiSlave(bus, cs, config) {};

    static SPIBusConfig getDefaultSPIConfig();

    /**
     * @brief Initialization Sequence, necessary to set all the registers to the
     * default settings.
     */
    void init();

    /**
     * @brief Set MIRROR value.
     * @param value 1 - The interrupt pins are internally connected;
     *              0 - The interrupt pins are not connected;
     */
    void setMIRROR(bool value);

    /**
     * @brief Set SEQOP (Sequential Operation) value.
     * @param value 1 - Sequential operation disabled;
     *              0 - Sequential operation enabled
     */
    void setSEQOP(bool value);

    /**
     * @brief Set DISSLW (Slew Rate Control for SDA Output) value.
     * @param value 1 - Slew rate disabled;
     *              0 - Slew rate enabled
     */
    void setDISSLW(bool value);

    /**
     * @brief Set HAEN (Hardware address enable) value.
     * @param value 1 - HA enabled;
     *              0 - HA disabled (device opcode is 000 for A0 A1 A2)
     */
    void setHAEN(bool value);

    /**
     * @brief Configure the INT pin as an open-drain output.
     * @param value 1 - Open drain Output (overrides INTPOL);
     *              0 - Active Driver Output (INTPOL Bit sets polarity)
     */
    void setODR(bool value);

    /**
     * @brief Configure polarity of the INT output pin.
     * @param value 1 - Active high;
     *              0 - Active low
     */
    void setINTPOL(bool value);

    /**
     * @brief Set the mode (input/ouput) of a pin.
     * @param port Port of the target pin
     * @param pinNumber Number of the target pin
     * @param mode Either INPUT, OUTPUT, or INPUT_PULL_UP
     *
     * @note Only pins set as input can be configured as pull-up
     */
    void setPinMode(MCP23S17Defs::PORT port, MCP23S17Defs::PIN pinNumber,
                    MCP23S17Defs::MODE mode);

    /**
     * @brief Set polarity of a pin.
     * @param port Port of the target pin
     * @param pinNumber Number of the target pin
     * @param polarity 1 - GPIO register bit reflects the inverted logic state
     * of the input pin; 0 - GPIO register bit reflects the same logic state of
     * the input pin
     */
    void setPinPolarity(MCP23S17Defs::PORT port, MCP23S17Defs::PIN pinNumber,
                        bool polarity);

    /**
     * @brief Enables interrupt on change for a pin. Note: DEFVAL and
     * INTCON registers must also be configured.
     * @param port Port of the target pin
     * @param pinNumber Number of the target pin
     */
    void enableInterruptOnChange(MCP23S17Defs::PORT port,
                                 MCP23S17Defs::PIN pinNumber);

    /**
     * @brief Disables interrupt on change for a pin.
     * @param port Port of the target pin
     * @param pinNumber Number of the target pin
     */
    void disableInterruptOnChange(MCP23S17Defs::PORT port,
                                  MCP23S17Defs::PIN pinNumber);

    /**
     * @brief Sets value to compare against when a pin is configured to trigger
     * an interrupt on change.
     * @param port Port of the target pin
     * @param pinNumber Number of the target pin
     * @param value Comparison value
     */
    void setDefaultValue(MCP23S17Defs::PORT port, MCP23S17Defs::PIN pinNumber,
                         bool value);

    /**
     * @brief Sets comparison mode for a pin set as an interrupt.
     * @param port Port of the target pin
     * @param pinNumber Number of the target pin
     * @param mode 1 - Compared against the corresponding value in DEFVAL;
     *             0 - Compared against the previous value
     */
    void setInterruptComparison(MCP23S17Defs::PORT port,
                                MCP23S17Defs::PIN pinNumber, bool mode);

    /**
     * @brief Read which pins caused an interrupt on a specific port.
     * @param port Port to check for interrupts
     * @return Snapshot of the pins that triggered the interrupt
     */
    uint8_t readInterruptFlag(MCP23S17Defs::PORT port);

    /**
     * @brief Read the value of the pins on a port when the interrupt got
     * triggered.
     * @return Snapshot of the pins when the interrupt was triggered
     */
    uint8_t readInterruptCapture(MCP23S17Defs::PORT port);

    /**
     * @brief Read the value of a specific pin.
     * @param port Port of the target pin
     * @param pinNumber Number of the target pin
     * @return Value stored in the target pin
     */
    bool getPinValue(MCP23S17Defs::PORT port, MCP23S17Defs::PIN pinNumber);

    /**
     * @brief Set the value of a specific pin
     * @param port Port of the target pin
     * @param pinNumber Number of the target pin
     * @param value Value to be set on the target pin
     */
    void setPinValue(MCP23S17Defs::PORT port, MCP23S17Defs::PIN pinNumber,
                     bool value);

private:
    /**
     * @brief Read all the values stored in a register.
     * @param address Register address
     * @return Byte stored in the target register
     */
    uint8_t readRegister(uint8_t address);

    /**
     * @brief Read a specific bit from a register.
     * @param address Register address
     * @param bitNumber Number of the target bit within the register (between 0
     * and 7)
     * @return Value stored in the target bit
     */
    bool readBit(uint8_t address, uint8_t bitNumber);

    /**
     * @brief Write a value in a specific bit of a register.
     * @param address Register address
     * @param bitNumber Number of the target bit within the register (between 0
     * and 7)
     * @param value Value to write
     */
    void writeBit(uint8_t address, uint8_t bitNumber, bool value);

    /**
     * @brief Write 8 bits in a specific register.
     * @param address Register address
     * @param value Byte of data to be written in the register
     */
    void writeRegister(uint8_t address, uint8_t value);

    SPISlave spiSlave;
    miosix::FastMutex spiMutex;
};

}  // namespace Boardcore
