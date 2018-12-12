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

static const PWM::Timer CUTTER_TIM{
    TIM4, &(RCC->APB1ENR), RCC_APB1ENR_TIM4EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

static const PWMChannel CUTTER_CHANNEL_DROGUE = PWMChannel::CH1;  // PD12
static const PWMChannel CUTTER_CHANNEL_MAIN_CHUTE = PWMChannel::CH2;  // PD13

typedef miosix::actuators::hbridger::ena DrogueCutterEna;
typedef miosix::actuators::hbridgel::ena MainChuteCutterEna;

static const unsigned int CUTTER_PWM_FREQUENCY = 150;
static const float CUTTER_PWM_DUTY_CYCLE       = 32.0f / 256;

/**
 * @brief Delay to allow a safe shutdown of the hbridge
 *
 */
static const int HBRIDGE_DISABLE_DELAY_MS = 50;
