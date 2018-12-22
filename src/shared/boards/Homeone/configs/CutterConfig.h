/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include "drivers/HardwareTimer.h"
#include "drivers/pwm/pwm.h"
#include "interfaces-impl/hwmapping.h"


namespace HomeoneBoard
{
// clang-format off

// Struct required by the PWM driver to know the specifics of the timer to use
static const PWM::Timer CUTTER_TIM{
    TIM4, 
    &(RCC->APB1ENR), 
    RCC_APB1ENR_TIM4EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)
    };

// clang-format on

// DROGUE --> Right H-Bridge, THCUT1 on theboard
static const PWMChannel CUTTER_CHANNEL_DROGUE = PWMChannel::CH1; // PD12
typedef miosix::actuators::hbridger::ena DrogueCutterEna; // PG2

// MAIN CHUTE --> Left H-Bridge, THCUT2 on theboard
static const PWMChannel CUTTER_CHANNEL_MAIN_CHUTE = PWMChannel::CH2; // PD13
typedef miosix::actuators::hbridgel::ena MainChuteCutterEna; //PD11

// PWM Frequency & duty-cycle
static const unsigned int CUTTER_PWM_FREQUENCY = 15000;
static const float CUTTER_PWM_DUTY_CYCLE       = 48.0f / 256;

// Period of time where the IN must be kept low before bringing ENA/INH low
static const int HBRIDGE_DISABLE_DELAY_MS = 50;
 
} // HomeoneBoard


