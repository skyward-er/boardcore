/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Silvano Seva
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef STM32F2_I2C_H
#define	STM32F2_I2C_H

#include <interfaces/arch_registers.h>
#include "board_settings.h"

namespace miosix {

/**
 * Driver for the I2C1 peripheral in STM32F2 and STM32F4 under Miosix
 */
class I2C1Driver
{
public:
    /**
     * \return an instance of this class (singleton)
     */
    static I2C1Driver& instance();
    
    /**
     * Initializes the peripheral. The only supported mode is 100KHz, master,
     * 7bit address. Note that there is no need to manually call this member
     * function as the constructor already inizializes the I2C peripheral.
     * The only use of this member function is to reinitialize the peripheral
     * if the microcontroller clock frequency or the APB prescaler is changed.
     */
    void init();
    
    /**
     * Send data to a device connected to the I2C bus
     * \param address device address (bit 0 is forced at 0)
     * \param data pointer with data to send
     * \param len length of data to send
     * \param sendStop if set to false disables the sending of a stop condition
     *                 after data transmission has finished
     * \return true on success, false on failure
     */
    bool send(unsigned char address, 
            const void *data, int len, bool sendStop = true);
            
    /**
     * Receive data from a device connected to the I2C bus
     * \param address device address (bit 0 is forced at 1) 
     * \param data pointer to a buffer where data will be received
     * \param len length of data to receive
     * \return true on success, false on failure
     */
    bool recv(unsigned char address, void *data, int len);
    
private:
    I2C1Driver(const I2C1Driver&);
    I2C1Driver& operator=(const I2C1Driver&);
    
    /**
     * Constructor. Initializes the peripheral except the GPIOs, that must be
     * set by the caller to the appropriate alternate function mode prior to
     * creating an instance of this class.
     * \param i2c pinter to the desired I2C peripheral, such as I2C1, I2C2, ...
     */
    I2C1Driver() { init(); }
    
    /**
     * Send a start condition
     * \param address 
     * \param immediateNak
     * \return 
     */
    bool start(unsigned char address, bool immediateNak=false);
    
    /**
     * Wait until until an interrupt occurs during the send start bit and
     * send address phases of the i2c communication.
     * \return true if the operation was successful, false on error
     */
    bool waitStatus1();
    
    /**
     * Flag used to indicate that the send function is not sending the stop
     * signal.
     */
    bool noStop;    
};

} //namespace miosix

#endif //STM32F2_I2C_H
