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

#include <drivers/MCP23S17/MCP23S17Defs.h>
#include <drivers/spi/SPIDriver.h>

namespace Boardcore
{

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
     * @brief Helper function to get the wanted GPIO register from the LUT.
     * @param reg Register name
     * @return Address of the target GPIO register
     */
    uint8_t getGpioAddress(MCP23S17Defs::GPIO_REG reg);

    /**
     * @brief Helper function to get the wanted control register from the LUT.
     * @param reg Register name
     * @return Address of the target control register
     */
    uint8_t getCtrlAddress(MCP23S17Defs::CTRL_REG reg);

    /**
     * @brief Helper function to write all zeroes in a register.
     * @param address Register address
     */
    void wipeRegister(uint8_t address);

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

    /**
     * @brief Set a specific Port A pin as an input pin.
     * @param pinNumber Number of the target pin
     */
    void setPinIn_A(uint8_t pinNumber);

    /**
     * @brief Set a specific Port B pin as an input pin.
     * @param pinNumber Number of the target pin
     */
    void setPinIn_B(uint8_t pinNumber);

    /**
     * @brief Set a specific Port A pin as an output pin.
     * @param pinNumber Number of the target pin
     */
    void setPinOut_A(uint8_t pinNumber);

    /**
     * @brief Set a specific Port B pin as an output pin.
     * @param pinNumber Number of the target pin
     */
    void setPinOut_B(uint8_t pinNumber);

    /**
     * @brief Set polarity of a Port A pin.
     * @param pinNumber Number of the target pin
     * @param polarity: 1 - GPIO register bit will reflet the opposite logic
     * state of the input pin; 0 - GPIO register bit will reflet the same logic
     * state of the input pin
     */
    void setPinPolarity_A(uint8_t pinNumber, bool polarity);

    /**
     * @brief Set polarity of a Port B pin.
     * @param pinNumber Number of the target pin
     * @param polarity 1 - GPIO register bit will reflet the opposite logic
     * state of the input pin; 0 - GPIO register bit will reflet the same logic
     * state of the input pin
     */
    void setPinPolarity_B(uint8_t pinNumber, bool polarity);

    /**
     * @brief Set BANK value.
     * @param value 1 - The registers associated with each port are separated
     * into different banks; 0 - The registers are in the same bank (addresses
     * are sequential)
     */
    void setBANK(bool value);

    /**
     * @brief Set MIRROR value.
     * @param value 1 - The INT pins are internally connected;
     *              0 - The INT pins are not connected; INTA refers to port A
     * and INTB refers to port B
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
     * @brief Enables interrupt on change for a Port A pin. Note: DEFVAL and
     * INTCON registers must also be configured.
     * @param pinNumber Number of the target pin
     */
    void enableInterruptOnChange_A(uint8_t pinNumber);

    /**
     * @brief Enables interrupt on change for a Port B pin. Note: DEFVAL and
     * INTCON registers must also be configured.
     * @param pinNumber Number of the target pin
     */
    void enableInterruptOnChange_B(uint8_t pinNumber);

    /**
     * @brief Disables interrupt on change for a Port A pin.
     * @param pinNumber Number of the target pin
     */
    void disableInterruptOnChange_A(uint8_t pinNumber);

    /**
     * @brief Disables interrupt on change for a Port B pin.
     * @param pinNumber Number of the target pin
     */
    void disableInterruptOnChange_B(uint8_t pinNumber);

    /**
     * @brief Enables pull up resistors for a Port A pin.
     * @param pinNumber Number of the target pin
     */
    void enablePullUp_A(uint8_t pinNumber);

    /**
     * @brief Enables pull up resistors for a Port B pin.
     * @param pinNumber Number of the target pin
     */
    void enablePullUp_B(uint8_t pinNumber);

    /**
     * @brief Disables pull up resistors for a Port A pin.
     * @param pinNumber Number of the target pin
     */
    void disablePullUp_A(uint8_t pinNumber);

    /**
     * @brief Disables pull up resistors for a Port B pin.
     * @param pinNumber Number of the target pin
     */
    void disablePullUp_B(uint8_t pinNumber);

    /**
     * @brief Sets default comparison value for a Port A pin set as an interrupt
     * pin.
     * @param value Comparison value
     */
    void setDefaultValue_A(uint8_t pinNumber, bool value);

    /**
     * @brief Sets default comparison value for a Port B pin set as an interrupt
     * pin.
     * @param value Comparison value
     */
    void setDefaultValue_B(uint8_t pinNumber, bool value);

    /**
     * @brief Sets comparison mode for a Port A pin set as an interrupt.
     * @param pinNumber Number of the target pin
     * @param mode 1 - Compared against the corresponding value in DEFVAL;
     *             0 - Compared against the previous value
     */
    void setInterruptComparison_A(uint8_t pinNumber, bool mode);

    /**
     * @brief Sets comparison mode for a Port B pin set as an interrupt.
     * @param pinNumber Number of the target pin
     * @param mode 1 - Compared against the corresponding value in DEFVAL;
     *             0 - Compared against the previous value
     */
    void setInterruptComparison_B(uint8_t pinNumber, bool mode);

    /**
     * @brief Read which pin in Port A caused an interrupt.
     * @return Number of the pin
     */
    uint8_t readInterruptFlag_A();

    /**
     * @brief Read which pin in Port B caused an interrupt.
     * @return Number of the pin
     */
    uint8_t readInterruptFlag_B();

    /**
     * @brief Read the value of the GPIO (Port A) when the interrupt got
     * triggered.
     * @return GPIO register values
     */
    uint8_t readInterruptCapture_A();

    /**
     * @brief Read the value of the GPIO (Port B) when the interrupt got
     * triggered.
     * @return GPIO register values
     */
    uint8_t readInterruptCapture_B();

    /**
     * @brief Read the value of a specific Port A pin.
     * @param pinNumber Number of the target pin
     * @return Pin value
     */
    bool readPin_A(uint8_t pinNumber);

    /**
     * @brief Read the value of a specific Port B pin.
     * @param pinNumber Number of the target pin
     * @return Pin value
     */
    bool readPin_B(uint8_t pinNumber);

    /**
     * @brief Read the value of a specific Port A latch (associated to a pin).
     * @param pinNumber Number of the pin related to the target latch
     * @return Latch value
     */
    bool readLatch_A(uint8_t pinNumber);

    /**
     * @brief Read the value of a specific Port B latch (associated to a pin).
     * @param pinNumber Number of the pin related to the target latch
     * @return Latch value
     */
    bool readLatch_B(uint8_t pinNumber);

    /**
     * @brief Write a value to a specific Port A pin - Note: Updates the value
     * of the associated latch (OLATA).
     * @param pinNumber Number of the target pin
     */
    void writePin_A(uint8_t pinNumber, bool value);

    /**
     * @brief Write a value to a specific Port B pin - Note: Updates the value
     * of the associated latch (OLATB).
     * @param pinNumber Number of the target pin
     */
    void writePin_B(uint8_t pinNumber, bool value);

    /**
     * @brief Write a value to a specific Port A latch - Note: Updates the value
     * of the associated GPIO pin (if configured as output).
     * @param pinNumber Number of the pin associated to the target latch
     */
    void writeLatch_A(uint8_t pinNumber, bool value);

    /**
     * @brief Write a value to a specific Port A latch - Note: Updates the value
     * of the associated GPIO pin (if configured as output).
     * @param pinNumber Number of the pin associated to the target latch
     */
    void writeLatch_B(uint8_t pinNumber, bool value);

private:
    SPISlave spiSlave;
    MCP23S17Defs::Bank activeBank = MCP23S17Defs::Bank::Bank0;
};

}  // namespace Boardcore
