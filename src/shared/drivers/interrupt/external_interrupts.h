/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <interfaces/gpio.h>

enum class InterruptTrigger
{
    RISING_EDGE,
    FALLING_EDGE,
    RISING_FALLING_EDGE
};

/**
 * @brief Enables external interrupts on the provided pin.
 * Remember to set the GPIO to input mode!
 *
 * @param gpioPort Port of the pin (eg:  GPIOC_BASE)
 * @param gpioNum Pin number (eg: 4 for PC4)
 * @param trigger Interrupt detection trigger (rising edge, falling or both)
 * @param priority Interrupt priority [0-15], 0 = Highest priority
 */
void enableExternalInterrupt(unsigned int gpioPort, unsigned int gpioNum,
                             InterruptTrigger trigger,
                             unsigned int priority = 15);

/**
 * @brief Enables external interrupts on the provided pin.
 * Remember to set the GPIO to input mode!
 *
 * @param gpio Pin (eg: PC4)
 * @param trigger Interrupt detection trigger (rising edge, falling or both)
 * @param priority Interrupt priority [0-15], 0 = Highest priority
 */
inline void enableExternalInterrupt(miosix::GpioPin gpio,
                                    InterruptTrigger trigger,
                                    unsigned int priority = 15)
{
    enableExternalInterrupt(gpio.getPort(), gpio.getNumber(), trigger,
                            priority);
}

/**
 * @brief Disables external interrupts on the provided pin.
 *
 * @param gpioPort Port of the pin (eg:  GPIOC_BASE)
 * @param gpioNum Pin number (eg: 4 for PC4)
 */
void disableExternalInterrupt(unsigned int gpioPort, unsigned int gpioNum);

/**
 * @brief Disables external interrupts on the provided pin.
 *
 * @param gpio Pin (eg: PC4)
 */
inline void disableExternalInterrupt(miosix::GpioPin gpio)
{
    disableExternalInterrupt(gpio.getPort(), gpio.getNumber());
}

/**
 * @brief Changes interrupt trigger on an enabled interrupt.
 *
 * @param gpioPort Port of the pin (eg:  GPIOC_BASE)
 * @param gpioNum Pin number (eg: 4 for PC4)
 * @param trigger Interrupt detection trigger (rising edge, falling or both)
 */
void changeInterruptTrigger(unsigned int gpioPort, unsigned int gpioNum,
                            InterruptTrigger trigger);

/**
 * @brief Changes interrupt trigger on an enabled interrupt.
 *
 * @param gpio Pin (eg: PC4)
 * @param trigger Interrupt detection trigger (rising edge, falling or both)
 */
inline void changeInterruptTrigger(miosix::GpioPin gpio,
                                   InterruptTrigger trigger)
{
    changeInterruptTrigger(gpio.getPort(), gpio.getNumber(), trigger);
}