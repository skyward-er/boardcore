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
 * @brief Low level driver for I2C peripherals.
 *
 * This is NOT a thread safe driver. The features supported are:
 * - Only Master logic;
 * - Standard/Fast speed modes;
 * - 7bit and 10bit addressing;
 * - Exposes basic read or write methods with the option for the write method to
 * not generate a stop condition;
 * - There is a method 'flushBus' in order to check and possibly recover from a
 * locked state on the bus;
 * - Dynamic setting of clock parameters in order to change speed or addressing
 * mode before interacting with a device;
 */
class I2CDriver
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
     * @brief Configuration struct for a slave device. This will be used for
     * configuring the bus in order to communicate with the addressed device.
     */
    typedef struct
    {
        uint16_t slaveAddress;             ///< Slave address
        I2CDriver::Addressing addressing;  ///< Addressing mode of the device
        I2CDriver::Speed speed;            ///< Speed mode of the communication
    } I2CSlaveConfig;

    /**
     * @brief Constructor for the I2C low-level driver.
     *
     * @param i2c Structure that represents the I2C peripheral.
     * @param scl Serial clock GpioPin of the relative I2C peripheral.
     * @param sda Serial data GpioPin of the relative I2C peripheral.
     */
    I2CDriver(I2C_TypeDef *i2c, miosix::GpioPin scl, miosix::GpioPin sda);

    /**
     * @brief Disables the peripheral, the interrupts in the NVIC and the
     * peripheral's clock.
     */
    ~I2CDriver();

    /**
     * @brief Non blocking read operation to read nBytes. In case of an error
     * during the communication, this method returns false immediately.
     *
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param buffer Data buffer where to store the data read from the bus.
     * @param nBytes Number of bytes to read.
     * @return True if the read is successful, false otherwise.
     */
    [[nodiscard]] bool read(I2CSlaveConfig slaveConfig, void *buffer,
                            size_t nBytes);

    /**
     * @brief Non blocking write operation to write nBytes. In case of an error
     * during the communication, this method returns false immediately.
     *
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param buffer Data buffer where to read the data to send.
     * @param nBytes number of bytes to send.
     * @param generateStop flag for the stop condition generation.
     * @return true if the write is successful, false otherwise.
     */
    [[nodiscard]] bool write(I2CSlaveConfig slaveConfig, const void *buffer,
                             size_t nBytes, bool generateStop = true);

    /**
     * @brief Performs the recovery from the locked state if necessary.
     *
     * It tries to recover from the locked state forcing (changing the mode of
     * the clock pin) N_SCL_BITBANG clock cycles and reinitializing the
     * peripheral. It usually takes less than 200us for 16 clocks forced in
     * standard mode.
     */
    void flushBus();

    /**
     * @brief Handles the interrupt for events of the specific peripheral.
     *
     * It just disables the interrupts of the peripheral and wakes the thread
     * up.
     * @warning This function should only be called by interrupts. No user code
     * should call this method.
     */
    void IRQhandleInterrupt();

    /**
     * @brief Handles the interrupt for the errors in the specific peripheral.
     *
     * It disables the interrupts of the peripheral, wakes the thread up, sets
     * the "error" software flag and resets the error flags in the register.
     * @warning This function should only be called by interrupts. No user code
     * should call this method.
     */
    void IRQhandleErrInterrupt();

private:
    /**
     * @brief Enables the peripheral clock and sets up various parameters.
     */
    void init();

    /**
     * @brief Sets up the I2C peripheral registers in order to communicate with
     * the speed and the addressing mode specified.
     * @param slaveConfig The configuration struct of the slave device.
     */
    void setupPeripheral(I2CSlaveConfig slaveConfig);

    /**
     * @brief Prologue of any read/write operation in master mode.
     *
     * It also detects locked states; in this case sets the lockedState flag to
     * true. Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @return True if prologue didn't have any error; False otherwise.
     */
    [[nodiscard]] bool prologue(I2CSlaveConfig slaveConfig);

    /**
     * @brief This waits until the thread isn't waken up by an I2C interrupt (EV
     * or ERR).
     *
     * It handles the waiting and yielding and the management of the flags for
     * the interrupts.
     * @warning This method should be called in a block where interrupts are
     * disabled.
     * @return True if waken up by an event, false if an error occurred.
     */
    inline bool IRQwaitForRegisterChange(
        miosix::FastInterruptDisableLock &dLock);

    /**
     * @brief This function has the logic to wake up and reschedule the thread
     * if it has a higher priority with relation to the one in current
     * execution.
     */
    inline void IRQwakeUpWaitingThread();

    I2C_TypeDef *i2c;
    uint8_t id;
    IRQn_Type irqnEv;
    IRQn_Type irqnErr;
    miosix::GpioPin scl;  ///< GpioPin of the serial clock pin
    miosix::GpioPin sda;  ///< GpioPin of the serial data pin

    bool error       = false;  ///< Flag that tells if an error occurred
    bool lockedState = false;  ///< Flag for locked state detection
    bool reStarting  = false;  ///< Flag true if not generated a STOP condition
    miosix::Thread *waiting = 0;  ///< Pointer to the waiting on receive thread

    PrintLogger logger = Logging::getLogger("i2c");
};

}  // namespace Boardcore