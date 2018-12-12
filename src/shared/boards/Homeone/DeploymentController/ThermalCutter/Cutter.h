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

#include <miosix.h>

#include "CutterConfig.h"
#include "drivers/pwm/pwm.h"

using miosix::GpioPin;
using miosix::Thread;

class Cutter
{
public:
    Cutter()
        : pwm(CUTTER_TIM, CUTTER_PWM_FREQUENCY),
          pin_enable_drogue(DrogueCutterEna::getPin()),
          pin_enable_main_chute(MainChuteCutterEna::getPin())
    {
        pin_enable_drogue.low();
        pin_enable_main_chute.low();


        // Start PWM with 0 duty cycle to keep IN pins low
        pwm.enableChannel(CUTTER_CHANNEL_DROGUE, 0.0f);
        pwm.enableChannel(CUTTER_CHANNEL_MAIN_CHUTE, 0.0f);

        pwm.start();
    }

    ~Cutter()
    {
        pwm.stop();
    }

    void startCutDrogue()
    {
        if (!cutting_drogue)
        {
            enableCutter(CUTTER_CHANNEL_DROGUE, pin_enable_drogue);
            cutting_drogue = true;
        }
    }

    void stopCutDrogue()
    {
        if (cutting_drogue)
        {
            disableCutter(CUTTER_CHANNEL_DROGUE, pin_enable_drogue);
            cutting_drogue = false;
        }
    }

    void startCutMainChute()
    {
        if (!cutting_main)
        {
            enableCutter(CUTTER_CHANNEL_MAIN_CHUTE, pin_enable_main_chute);
            cutting_main = true;
        }
    }

    void stopCutMainChute()
    {
        if (cutting_main)
        {
            disableCutter(CUTTER_CHANNEL_MAIN_CHUTE, pin_enable_main_chute);
            cutting_main = false;
        }
    }

private:
    void enableCutter(PWMChannel channel, miosix::GpioPin& ena_pin)
    {
        // Enable PWM Generation
        pwm.setDutyCycle(channel, CUTTER_PWM_DUTY_CYCLE);

        // enable hbridge
        ena_pin.high();
    }

    void disableCutter(PWMChannel channel, miosix::GpioPin& ena_pin)
    {
        pwm.setDutyCycle(channel, 0.0f);  // Set duty cycle to 0 to leave the IN
                                          // pin of the Half-H bridge low

        Thread::sleep(HBRIDGE_DISABLE_DELAY_MS);  // Wait a short delay

        ena_pin.low();  // Disable hbridge
    }

    PWM pwm;
    GpioPin pin_enable_drogue;
    GpioPin pin_enable_main_chute;

    bool cutting_main   = false;
    bool cutting_drogue = false;
};