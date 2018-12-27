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
#include <events/EventBroker.h>

#include <boards/Nosecone/Events.h>
#include <boards/Nosecone/Topics.h>

#include "MotorDriver.h"
#include "MotorLimit.h"

using namespace miosix;

namespace NoseconeBoard
{


MotorDriver::MotorDriver(PinObserver* pinObs): pwm(MOTOR_TIM, MOTOR_PWM_FREQUENCY)
{
    /* Enable pins */
    RightEnable::mode(Mode::OUTPUT);
    LeftEnable::mode(Mode::OUTPUT);
    RightEnable::low();
    LeftEnable::low();
    
    /* Start PWM with 0 duty cycle to keep IN pins low */
    pwm.enableChannel(MOTOR_CH_RIGHT, 0.0f);
    pwm.enableChannel(MOTOR_CH_LEFT,  0.0f);
    pwm.start();

    /* Start observing motor limit pins */
    MotorLimit::observeLimitPins(pinObs);

    /* Start sampling motor current */
    currentSensor.start();

    /* Set status */
    status.motor_active = 0;
    log();
}


MotorDriver::~MotorDriver()
{
    pwm.stop();
}


bool MotorDriver::start(MotorDirection direction, float dutyCycle)
{   
    /* Read from status if the motor is active */
    if(status.motor_active == true) 
        return false;

    /* If it's not active, check the direction */
    bool dir = (direction == MotorDirection::NORMAL_DIRECTION);

    if(dir) {
        pwm.enableChannel(MOTOR_CH_RIGHT, dutyCycle, PWMMode::MODE_1, 
                                                        PWMPolarity::ACTIVE_HIGH);
        pwm.enableChannel(MOTOR_CH_LEFT,  dutyCycle, PWMMode::MODE_1, 
                                                        PWMPolarity::ACTIVE_LOW);
    } 
    else {
        pwm.enableChannel(MOTOR_CH_RIGHT, dutyCycle, PWMMode::MODE_1, 
                                                        PWMPolarity::ACTIVE_LOW);
        pwm.enableChannel(MOTOR_CH_LEFT,  dutyCycle, PWMMode::MODE_1, 
                                                        PWMPolarity::ACTIVE_HIGH);
    }
    
    /* Activate PWM on both half bridges */
    RightEnable::high();
    LeftEnable::high();

    /* Update status */
    status.motor_active = true;
    status.motor_last_direction = dir;
    log();

    return true;
}

void MotorDriver::stop()
{
    pwm.setDutyCycle(MOTOR_CH_LEFT, 0.0f);   // Set duty cycle to 0
    pwm.setDutyCycle(MOTOR_CH_RIGHT, 0.0f);  // Set duty cycle to 0

    Thread::sleep(HBRIDGE_DISABLE_DELAY_MS);  // Wait a short delay

    RightEnable::low();  // Disable hbridge
    LeftEnable::low(); 

    status.motor_active = false;
    log();
}

}