/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <diagnostic/PrintLogger.h>

using I2CType = I2C_TypeDef;

#if defined(STM32F429xx) || defined(STM32F407xx)
#define N_I2C_PORTS 3
#else
#define N_I2C_PORTS 1
#warning "Define the number of I2C ports for your architectures in I2C.h file"
#endif

namespace Boardcore
{

/**
 * Features implemented:
 * - Functions for all the I2C peripherals
 * - setting Speed modes Standard (100kbps) and Fast (400kbps, with dutycycle
 * hardcoded to 1-2 and not 9-16)
 *
 *
 * Features to be implemented:
 * - General call address detection
 * - DMA
 * - analog noise filter
 * - Programmable digital noise filter
 * - PEC (Packet Error Checking)
 *
 */
class I2C
{
public:
    enum Speed : uint8_t
    {
        STANDARD = 0,
        FAST     = 1
    };

    enum Mode
    {
        MASTER,
        SLAVE
    };

    enum Addressing : uint8_t
    {
        BIT7  = 0,
        BIT10 = 1
    };

    ///< Pointer to serial port classes to let interrupts access the classes
    static I2C *ports[];

    /**
     * @param speed the speed mode of the I2C communication
     * @param addressing The addressing mode used in the I2C communication
     * @param address In 7-bit addressing must give the 7 bits shifted to the
     * left by 1 (so, MSB of the address must be the eighth bit). In 10 bit
     * addressing, just the 10 bit address
     */
    I2C(I2CType *i2c, Speed speed, Addressing addressing, uint16_t address);

    ~I2C();

    /**
     * @brief Initializes the peripheral enabling his interrupts, the interrupts
     * in the NVIC.
     *
     * All the setup phase (with the setting of the pins and their alternate
     * functions) must be done before the initialization of the peripheral.
     */
    bool init();

    /**
     * @brief Blocking read operation to read nBytes.
     */
    int read(uint16_t slaveAddress, void *buffer, size_t nBytes);

    /**
     * @brief Blocking write operation.
     */
    int write(uint16_t slaveAddress, void *buf, size_t nChars);

    void IRQhandleInterrupt();

protected:
    bool prologue(uint16_t slaveAddress, bool writeOperation);

    uint8_t id;
    IRQn_Type irqnEv;
    IRQn_Type irqnErr;

    I2CType *i2c;
    bool initialized = false;
    const Speed speed;            ///< Baudrate of the serial communication
    const Addressing addressing;  ///< Addressing mode of the device
    const uint8_t header;  ///< Header generated (composed of 11110xx0 bits with
                           ///< xx as the 9th and 8th bits of the address)
    const uint16_t address;  ///< Address of the device
    PrintLogger logger = Logging::getLogger("i2c");

    /* handling of interrupts */
    miosix::Thread *waiting = 0;  ///< Pointer to the waiting on receive thread
};
}  // namespace Boardcore