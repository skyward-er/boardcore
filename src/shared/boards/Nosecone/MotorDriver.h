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

#pragma once

#include <Common.h>

#include <drivers/pwm/pwm.h>
#include <drivers/HardwareTimer.h>
#include <interfaces-impl/hwmapping.h>


namespace NoseconeBoard
{

/**
 * @brief PWM channel output polarity
 *
 */
enum class MotorDirection
{
    NORMAL_DIRECTION,
    REVERSE_DIRECTION
};


/* Struct required by the PWM driver to know the specifics of the timer to use */
static const PWM::Timer MOTOR_TIM {
    TIM4, 
    &(RCC->APB1ENR), 
    RCC_APB1ENR_TIM4EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)
    };

/* Pins definition */
static const PWMChannel MOTOR_CH_RIGHT = PWMChannel::CH1; // PD12
static const PWMChannel MOTOR_CH_LEFT  = PWMChannel::CH2; // PD13
typedef miosix::actuators::hbridger::ena RightEnable;     // PG2
typedef miosix::actuators::hbridgel::ena LeftEnable;      // PD11

/* PWM Frequency & duty-cycle */
static const unsigned int MOTOR_PWM_FREQUENCY = 150;

/* Period of time where the IN must be kept low before bringing ENA/INH low */
static const int HBRIDGE_DISABLE_DELAY_MS = 50;

/**
 * This class gives access to the H-Bridge that controls the DC motor of the nosecone.
 */
class MotorDriver
{

public:
    MotorDriver();
    ~MotorDriver();

    /**
     * @brief Activates the H-Bridge to start the motor.
     *
     * @param direction     direction of activation (normal or reservse)
     * @param dutyCycle     duty cycle of the PWM sent to the motor
     * @return              false if the motor is already started
     */
    bool start(MotorDirection direction, float dutyCycle);

    /**
     * @brief Stop the motor
     */
    void stop();

    // TODO: MotorStatus getStatus();

private:
    PWM pwm;
    bool active;   
};

} /* namespace  */