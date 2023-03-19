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
 * - 7bit addressing;
 * - Exposes basic read or write methods with the option for the write method to
 * not generate a STOP condition;
 * - There is a method 'flushBus' in order to check and possibly recover from a
 * locked state on the bus;
 * - Dynamic setting of clock parameters in order to change speed or addressing
 * mode before interacting with a device;
 */
class I2CDriver
{
public:
    enum Operation : uint8_t
    {
        WRITE = 0,
        READ  = 1
    };

    // [TODO] limit speed possibilities at compile time with ifdefs?
    enum Speed : uint8_t
    {
        STANDARD  = 0,
        FAST      = 1,
        FAST_PLUS = 2
    };

    enum Addressing : uint8_t
    {
        BIT7  = 0,
        BIT10 = 1
    };

    /**
     * @brief Error enums with a value that makes it possible to or them and
     * report more than one error at once
     */
    enum Errors : uint16_t
    {
        NO_ERROR    = 0,       ///< The bus didn't have any error
        BUS_LOCKED  = 1 << 0,  ///< Detected a locked state on the bus
        BERR        = 1 << 1,  ///< External Start or stop condition detected
        ARLO        = 1 << 2,  ///< Arbitration lost
        AF          = 1 << 3,  ///< Acknowledge failure
        OVR         = 1 << 4,  ///< Overrun/underrun error
        SB_NOT_SENT = 1 << 5,  ///< Start bit not sent
        ADDR_ERROR  = 1 << 6   ///< Address sent but peripheral in wrong state
    };

    /**
     * @brief Configuration struct for a slave device. This will be used for
     * configuring the bus in order to communicate with the addressed device.
     */
    typedef struct
    {
        uint16_t
            slaveAddress;  ///< Slave address without shifts (in BIT7 addressing
                           ///< |9-bit unused|7-bit address|; in BIT10
                           ///< addressing |6-bit unused|10-bit address|).
        I2CDriver::Addressing addressing;  ///< Addressing mode of the device.
        I2CDriver::Speed speed;            ///< Speed mode of the communication.
    } I2CSlaveConfig;

    /**
     * @brief Constructor for the I2C low-level driver.
     *
     * It initializes the peripheral clock, the pins, calls the `init()` method
     * and enables the IRQs in the NVIC.
     * Pins are internally initialized so that they are always set to
     * ALTERNATE_OD mode with Alternate Function 4 (the usual AF of I2C pins).
     * Thanks to this we avoid the possibility of short circuits between master
     * and slaves when they both drive the same bus on two different logical
     * values.
     *
     * @param i2c Structure that represents the I2C peripheral.
     * @param scl Serial clock GpioPin of the relative I2C peripheral.
     * @param sda Serial data GpioPin of the relative I2C peripheral.
     */
    I2CDriver(I2C_TypeDef *i2c, miosix::GpioPin scl, miosix::GpioPin sda);

    ///< Delete copy/move constructors/operators.
    I2CDriver(const I2CDriver &)            = delete;
    I2CDriver &operator=(const I2CDriver &) = delete;
    I2CDriver(I2CDriver &&)                 = delete;
    I2CDriver &operator=(I2CDriver &&)      = delete;

    /**
     * @brief Disables the peripheral, the interrupts in the NVIC and the
     * peripheral's clock.
     */
    ~I2CDriver();

    /**
     * @brief Read operation to read nBytes. In case of an error during the
     * communication, this method returns false with no further attempts.
     *
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param buffer Data buffer where to store the data read from the bus.
     * @param nBytes Number of bytes to read.
     * @return True if the read is successful, false otherwise.
     */
    [[nodiscard]] bool read(const I2CSlaveConfig &slaveConfig, void *buffer,
                            const size_t &nBytes);

    /**
     * @brief Write operation to write nBytes. In case of an error during the
     * communication, this method returns false with no further attempts.
     *
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @param buffer Data buffer where to read the data to send.
     * @param nBytes Number of bytes to send.
     * @param generateStop Flag for the stop condition generation.
     * @return True if the write is successful, false otherwise.
     */
    [[nodiscard]] bool write(const I2CSlaveConfig &slaveConfig,
                             const void *buffer, const size_t &nBytes,
                             bool generateStop = true);

    /**
     * @brief Performs the recovery from the locked state if necessary.
     *
     * It tries to recover from the locked state forcing (changing the mode of
     * the clock pin) N_SCL_BITBANG clock cycles and re-initializing the
     * peripheral. It usually takes less than 200us for 16 clocks forced in
     * standard mode.
     */
    void flushBus();

    /**
     * @brief Returns the last errors happened in the communication.
     *
     * For checking if a specific error occurred in the last transaction you can
     * do `if(getLastError() & Errors::<error-to-check>)`. Do not use `==` to
     * check for errors because there could be more errors at once. To check if
     * no errors occurred use `if(getLastError() == Errors::NO_ERROR)` or simply
     * `if(!getLastError())`
     *
     * @return A bit sequence where the bits set correspond to the last errors
     * occurred in the peripheral (see the `I2CDriver::Errors` enum to get the
     * correspondence between bit position and error reported).
     */
    uint16_t getLastError();

    /**
     * @brief Handles the interrupt for events of the specific peripheral.
     *
     * Wakes up the thread only if the operation is completed or an error is
     * detected, otherwise all the phases of the read or write are handled in
     * this ISR thanks to the changing of the peripheral flags.
     *
     * @warning This function should only be called by interrupts. No user code
     * should call this method.
     */
    void IRQhandleInterrupt();

    /**
     * @brief Handles the interrupt for the errors in the specific peripheral.
     *
     * It disables the interrupts of the peripheral, wakes the thread up, sets
     * the "error" software flag and resets the error flags in the register.
     *
     * @warning This function should only be called by interrupts. No user code
     * should call this method.
     */
    void IRQhandleErrInterrupt();

private:
    /**
     * @brief Structure that stores all the info of the transaction, such as
     * operation, buffers, number of bytes of the buffer, number of bytes
     * already processed (read or written) and whether to generate the stop bit
     * or not.
     */
    typedef struct
    {
        Operation operation;       ///< Operation to be performed (R/W)
        uint8_t *buffRead;         ///< Buffer with the data to read
        const uint8_t *buffWrite;  ///< Buffer with the data to write
        size_t nBytes;             ///< Number of bytes of the buffer
        size_t nBytesDone;         ///< Number of bytes already processed
        bool generateStop;         ///< Whether to generate stop condition
    } I2CTransaction;

    /**
     * @brief Resets the peripheral and sets up the internal clock parameter
     * parameters.
     */
    void init();

    /**
     * @brief Sets up the I2C peripheral registers in order to communicate with
     * the speed and the addressing mode specified.
     * @param slaveConfig The configuration struct of the slave device.
     */
    void setupPeripheral(const I2CSlaveConfig &slaveConfig);

#ifdef _ARCH_CORTEXM7_STM32F7
    inline void setupTransaction();

    inline void setupReload();
#endif  // _ARCH_CORTEXM7_STM32F7

    /**
     * @brief Method to perform a read or write operation.
     *
     * This method waits for the bus to be clear, sets up the peripheral for the
     * new communication, sends the START condition and the address. After this,
     * it delegates the logic to the event ISR.
     *
     * @warning Check always if the operation succeeded or not!
     * @param slaveConfig The configuration struct of the slave device.
     * @return True if the operation succeeded, False otherwise.
     */
    [[nodiscard]] bool doOperation(const I2CSlaveConfig &slaveConfig);

    /**
     * @brief This waits until the thread isn't waken up by an I2C interrupt (EV
     * or ERR).
     *
     * It handles the waiting and yielding and the management of the flags for
     * the interrupts.
     *
     * @warning This method should be called in a block where interrupts are
     * disabled.
     * @param dLock Reference to the InterruptDisableLock object active in the
     * scope.
     * @return True if waken up by an event, false if an error occurred.
     */
    inline bool IRQwaitForOperationCompletion(
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

    uint16_t lastError = NO_ERROR;  ///< Flag for the last error occurred
    uint32_t error     = 0;         ///< Flag that tells if an error occurred
    bool reStarting = false;    ///< Flag true if not generated a STOP condition
    miosix::Thread *waiting{};  ///< Pointer to the waiting for event thread
    I2CTransaction transaction;  ///< Struct storing the transaction info

    PrintLogger logger = Logging::getLogger("i2c");
};

}  // namespace Boardcore