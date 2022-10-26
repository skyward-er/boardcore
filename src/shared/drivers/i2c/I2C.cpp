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

#include "I2C.h"

#include <utils/ClockUtils.h>

/**
 * I2C address sent interrupt
 */
void __attribute__((naked)) I2C1_EV_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z15I2C1HandlerImplv");
    restoreContext();
}

/**
 * I2C address sent interrupt actual implementation
 */
void __attribute__((used)) I2C1HandlerImpl() {}

/**
 * I2C error interrupt
 */
void __attribute__((naked)) I2C1_ER_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z18I2C1errHandlerImplv");
    restoreContext();
}

/**
 * I2C error interrupt actual implementation
 */
void __attribute__((used)) I2C1errHandlerImpl() {}

namespace Boardcore
{

I2C::I2C(I2CType *i2c, Speed speed, Addressing addressing, uint16_t address)
    : i2c(i2c), speed(speed), addressing(addressing), address(address)
{
}

I2C::~I2C() {}

bool I2C::init()
{
    /* setting the peripheral input clock */
    // frequency of the APB relative to the I2C peripheral in MHz
    uint32_t f = ClockUtils::getAPBFrequency(ClockUtils::APB::APB1) >>
                 20;  // I2C peripherals are always connected to
                      // APB1 (Low speed bus)

    // frequency higher than 50MHz not allowed
    if (f > 50)
    {
        LOG_ERR(logger, "I2C error: APB frequency too high!");
        return false;
    }

    // frequency < 2MHz in standard mode or < 4MHz in fast mode not allowed
    if ((speed == STANDARD && f < 2) || (speed == FAST && f < 4))
    {
        LOG_ERR(logger, "I2C error: APB frequency too low!");
        return false;
    }

    if (i2c->CR1 & I2C_CR1_PE)
    {
        LOG_ERR(logger,
                "I2C error: Can't set CCR if the peripheral is enabled!");
        return false;
    }

    i2c->CCR |= (speed ? I2C_CCR_FS : 0);  // setting F/S speed bits

    if (!speed)
    {
        // if STANDARD mode, this is the divider to the peripheral clock to
        // reach the wanted frequency; It's divided by 2 because in reality it
        // uses this value to calculate the time that the clock needs to be in
        // the "set" state. [* 1000 KHz / (100 KHz * 2) = *5]
        i2c->CCR |= f * 5;  // setting the CCR bits
    }
    else
    {
        // [WARNING] hardcocded to use DUTY = 0
        i2c->CCR |= f * 10 / 3;  // setting the CCR bits

        // for DUTY = 1
        // i2c->CCR |= f * 10 / 25;  // setting the CCR bits
        // i2c->CCR |= f * 2 / 5;  // setting the CCR bits
    }

    i2c->CR2 = 0;
    i2c->CR2 |= (f & I2C_CR2_FREQ);  // setting FREQ bits

    /* Setting the Own Address Register */
    i2c->OAR1 = 0;
    i2c->OAR1 |= (addressing << 15);
    i2c->OAR1 |= (address << 1);

    return true;
}

/**
 * @brief Blocking read operation to read nBytes or till the data transfer
 * is complete.
 */
int I2C::read(void *buffer, size_t nBytes) { return 0; };

/**
 * @brief Blocking write operation.
 */
int I2C::write(void *buf, size_t nChars) { return 0; };
}  // namespace Boardcore