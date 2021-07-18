/* Copyright (c) 2018 Skyward Experimental Rocketry
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

#ifndef SRC_SHARED_DRIVERS_PWM_PWMDATA_H
#define SRC_SHARED_DRIVERS_PWM_PWMDATA_H

/**
 * @brief PWM channel output polarity
 *
 */
enum class PWMPolarity
{
    ACTIVE_HIGH,
    ACTIVE_LOW
};

/**
 * @brief PWM mode selection. Refer to datasheet
 * MODE_1  Channel high when CNT < CCRx
 * MODE_2  Channel high when CNT > CCRx
 */
enum class PWMMode
{
    MODE_1,  // Channel high when CNT < CCRx
    MODE_2   // Channel high when CNT > CCRx
};

enum class PWMChannel : int
{
    CH1 = 0,
    CH2,
    CH3,
    CH4
};

struct PWMChannelConfig
{
    PWMChannel channel;
    bool enabled = false;
    float duty_cycle = 0;
    PWMMode mode = PWMMode::MODE_1;
    PWMPolarity polarity = PWMPolarity::ACTIVE_HIGH;
};

#endif