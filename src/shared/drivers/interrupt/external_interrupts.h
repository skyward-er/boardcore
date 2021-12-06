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
 * @param gpio_port Port of the pin (eg:  GPIOC_BASE)
 * @param gpio_num Pin number (eg: 4 for PC4)
 * @param trigger Interrupt detection trigger (rising edge, falling or both)
 * @param priority Interrupt priority [0-15], 0 = Highest priority
 */
void enableExternalInterrupt(unsigned int gpio_port, unsigned int gpio_num,
                     InterruptTrigger trigger, unsigned int priority = 15);
