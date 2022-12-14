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

#include "miosix.h"

#if defined(I2C4)
#define N_I2C_PORTS 4
#elif defined(I2C3)
#define N_I2C_PORTS 3
#elif defined(I2C2)
#define N_I2C_PORTS 2
#elif defined(I2C1)
#define N_I2C_PORTS 1
#else
#error "Your architecture doesn't support I2C"
#endif

namespace Boardcore
{

/**
 * @brief Low driver for I2C peripherals. This is NOT thread safe. It
 * implements only the Master logic and supports Standard/Fast speed modes and
 * 7bit/10bit addressing. Exposes only base read/write methods and a flushBus
 * method to be called by the user when we want to check if the bus is locked
 * and, in this case, tries to recover it.
 */
class I2C
{
public:
    enum Speed : uint8_t
    {
        STANDARD = 0,
        FAST     = 1
    };

    enum Addressing : uint8_t
    {
        BIT7  = 0,
        BIT10 = 1
    };

    /**
     * @param i2c structure that represents the I2C peripheral
     * @param speed the speed mode of the I2C communication
     * @param addressing The addressing mode used in the I2C communication
     */
    I2C(I2C_TypeDef *i2c, Speed speed, Addressing addressing,
        miosix::GpioPin scl, miosix::GpioPin sda);

    /**
     * @brief Deconstructor. Disables the peripheral, the interrupts in the NVIC
     * and the clock of the peripheral.
     */
    ~I2C();

    /**
     * @brief Non blocking read operation to read nBytes. In case of an error
     * during the communication, this method returns false immediately. Check
     * always if the operation succeeded or not!
     * @param slaveAddress address (not shifted!) of the slave to communicate
     * with.
     * @param buffer Data buffer where to store the data read.
     * @param nBytes number of bytes to read.
     * @returns true if the read is successful, false otherwise.
     */
    [[nodiscard]] bool read(uint16_t slaveAddress, void *buffer, size_t nBytes);

    /**
     * @brief Non blocking write operation to write nBytes. In case of an error
     * during the communication, this method returns false immediately. Check
     * always if the operation succeeded or not!
     * @param slaveAddress address (not shifted!) of the slave to communicate
     * with.
     * @param buffer Data buffer where to read the data to send.
     * @param nBytes number of bytes to send.
     * @returns true if the write is successful, false otherwise.
     */
    [[nodiscard]] bool write(uint16_t slaveAddress, const void *buffer,
                             size_t nBytes);

    /**
     * @brief Performs the recovery from the locked state if necessary.
     * It tries to recover from the locked state forcing (changing the mode of
     * the clock pin) N_SCL_BITBANG clock cycles and reinitializing the
     * peripheral. It usually takes less than 200us for 16 clocks forced in
     * standard mode.
     */
    void flushBus();

    /**
     * @brief Method that handles the interrupt for events of the specific
     * peripheral. It just disables the interrupts of the peripheral and wakes
     * the thread up.
     * @warning No user code should call this method.
     */
    void IRQhandleInterrupt();

    /**
     * @brief Method that handles the interrupt for the errors in the specific
     * peripheral. It disables the interrupts of the peripheral, wakes
     * the thread up, sets the "error" software flag and resets the error flags
     * in the register.
     * @warning No user code should call this method.
     */
    void IRQhandleErrInterrupt();

private:
    /**
     * @brief Initializes the peripheral enabling his clock and sets up
     * various parameters in the peripheral. Safe to call also after init has
     * already been initialized in order to re-initialize the peripheral.
     */
    void init();

    /**
     * @brief Prologue of any read/write operation in Master mode. It also
     * detects locked states; in this case sets the lockedState flag to true.
     * Check always if the operation succeeded or not!
     * @param slaveAddress address (not shifted!) of the slave to communicate
     * with.
     * @returns True if prologue didn't have any error; False otherwise.
     */
    [[nodiscard]] bool prologue(uint16_t slaveAddress);

    /**
     * @brief This waits until the thread isn't waken up by an I2C interrupt (EV
     * or ERR). This method shuould be called in a block where interrupts are
     * disabled; handles the waiting and yielding and the management of the
     * flags for the interrupts.
     */
    inline bool IRQwaitForRegisterChange(
        miosix::FastInterruptDisableLock &dLock);

    /**
     * @brief This function has the logic to wake up and reschedule the thread
     * if it has a higher priority with relation to the one in current
     * execution.
     * @warning This function should only be called by interrupts.
     */
    inline void IRQwakeUpWaitingThread();

    I2C_TypeDef *i2c;
    uint8_t id;
    IRQn_Type irqnEv;
    IRQn_Type irqnErr;
    const Speed speed;            ///< Baudrate of the serial communication
    const Addressing addressing;  ///< Addressing mode of the device
    miosix::GpioPin scl;
    miosix::GpioPin sda;

    bool error              = false;  ///< Flag that tells if an error occurred
    bool lockedState        = false;  ///< Flag for locked state detection
    miosix::Thread *waiting = 0;  ///< Pointer to the waiting on receive thread

    PrintLogger logger = Logging::getLogger("i2c");
};

/**
 * @brief Thread safe version of the I2C driver.
 */
class SyncedI2C : public I2C
{
public:
    SyncedI2C(I2C_TypeDef *i2c, Speed speed, Addressing addressing,
              miosix::GpioPin scl, miosix::GpioPin sda);

    [[nodiscard]] bool read(uint16_t slaveAddress, void *buffer, size_t nBytes);

    [[nodiscard]] bool write(uint16_t slaveAddress, const void *buffer,
                             size_t nBytes);

    void flushBus();

private:
    miosix::FastMutex mutex;  ///< mutex for rx/tx
};

}  // namespace Boardcore