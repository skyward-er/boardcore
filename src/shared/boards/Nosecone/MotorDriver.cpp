/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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
#include <Common.h>
#include "MotorDriver.h"

namespace NoseconeBoard
{

using namespace miosix;

MotorDriver::MotorDriver(): pwm(MOTOR_TIM, MOTOR_PWM_FREQUENCY)
{
    RightEnable::low();
    LeftEnable::low();

    /* Start PWM with 0 duty cycle to keep IN pins low */
    pwm.enableChannel(MOTOR_CH_RIGHT, 0.0f);
    pwm.enableChannel(MOTOR_CH_LEFT,  0.0f);

    pwm.start();
}

MotorDriver::~MotorDriver()
{
    pwm.stop();
}


bool MotorDriver::start(MotorDirection direction, float dutyCycle)
{   
    if(active) {
        return false;
    } else {
        PWMPolarity rightPolarity  = (direction == MotorDirection::NORMAL_DIRECTION) ?
                                    PWMPolarity::ACTIVE_HIGH : PWMPolarity::ACTIVE_LOW;
        PWMPolarity leftPolarity   = (direction == MotorDirection::NORMAL_DIRECTION) ?
                                    PWMPolarity::ACTIVE_LOW : PWMPolarity::ACTIVE_HIGH;

        pwm.enableChannel(MOTOR_CH_RIGHT, dutyCycle, PWMMode::MODE_1, rightPolarity);
        pwm.enableChannel(MOTOR_CH_LEFT,  dutyCycle, PWMMode::MODE_1, leftPolarity);
        RightEnable::high();
        LeftEnable::high();

        active = true;
        return active;
    }
}

void MotorDriver::stop()
{
    pwm.setDutyCycle(MOTOR_CH_LEFT, 0.0f);   // Set duty cycle to 0
    pwm.setDutyCycle(MOTOR_CH_RIGHT, 0.0f);  // Set duty cycle to 0

    Thread::sleep(HBRIDGE_DISABLE_DELAY_MS);  // Wait a short delay

    RightEnable::low();  // Disable hbridge
    LeftEnable::low(); 
}

}