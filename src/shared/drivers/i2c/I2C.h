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
 * Thread safe driver for all I2C peripherals. It implements only the Master
 * logic and supports Standard/Fast speed modes and 7bit/10bit addressing.
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
        miosix::GpioPin scl, unsigned char af);

    /**
     * @brief Deconstructor. Disables the peripheral, the interrupts in the NVIC
     * and the clock of the peripheral.
     */
    ~I2C();

    /**
     * @brief Blocking read operation to read nBytes. In case of an error during
     * the communication, this method returns 0 immediately.
     * @returns true if the read is successful, false otherwise.
     */
    bool read(uint16_t slaveAddress, void *buffer, size_t nBytes,
              bool generateStopSignal);

    /**
     * @brief Blocking write operation of nBytes. In case of an error during the
     * communication, this method returns 0 immediately.
     * @returns true if the write is successful, false otherwise.
     */
    bool write(uint16_t slaveAddress, void *buffer, size_t nBytes,
               bool generateStopSignal);

    /**
     * @brief Method that handles the interrupt for the specific peripheral. It
     * just wakes the thread up
     */
    void IRQhandleInterrupt();

    /**
     * @brief Method that handles the interrupt for the errors in the specific
     * peripheral. It wakes the thread up, sets the "error" software flag and
     * resets the error flags in the register
     */
    void IRQhandleErrInterrupt();

protected:
    /**
     * @brief Initializes the peripheral enabling his clock, the interrupts
     * in the NVIC and setting up various parameters in the peripheral.
     */
    void init();

    /**
     * @brief Prologue of any read/write operation in Master mode.
     * @param address the 7 bit address NOT shifted
     * @returns True if prologue didn't have any error; False otherwise.
     */
    bool prologue(uint16_t slaveAddress);

    /**
     * @brief Performs the recovery from the locked state.
     * It tries to recover from both Master locked state and Slave locked state.
     * It *usually* takes circa 15ms for a full recovery.
     */
    void recoverFromLockedState();

    /**
     * @brief This waits until the thread isn't waken up by an I2C interrupt (EV
     * or ERR). This method shuould be called in a block where interrupts are
     * disabled; handles the waiting and yielding and the management of the
     * flags for the interrupts.
     */
    inline bool IRQwaitForRegisterChange(miosix::InterruptDisableLock &dLock);

    /**
     * @brief This function has the logic to wake up and reschedule the thread
     * if it has a higher priority with relation to the one in current
     * execution. This function is called by interrupts.
     * @param waiting The thread to be waken up.
     */
    inline void IRQwakeUpWaitingThread();

    uint8_t id;
    IRQn_Type irqnEv;
    IRQn_Type irqnErr;

    I2C_TypeDef *i2c;
    bool error = false;           ///< Flag that tells if an error occurred
    const Speed speed;            ///< Baudrate of the serial communication
    const Addressing addressing;  ///< Addressing mode of the device
    miosix::Thread *waiting = 0;  ///< Pointer to the waiting on receive thread
    miosix::GpioPin scl;          ///< Pin of the clock for slave lock recovery
    unsigned char af;             ///< Alternate function of the scl for i2c
    miosix::FastMutex mutex;      ///< recursive mutex for rx/tx

    PrintLogger logger = Logging::getLogger("i2c");
};
}  // namespace Boardcore