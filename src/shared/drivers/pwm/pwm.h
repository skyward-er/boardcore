/* Copyright (c) 2017-2018 Skyward Experimental Rocketry
 * Authors: Andrea Palumbo, Luca Erbetta
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

#include <miosix.h>

#include "pwmData.h"

namespace Boardcore
{

/**
 * @brief Class to generate PWM using hardware timers.
 * Class is NOT thread-safe. Every instante must be used by a single thread.
 */
class PWM
{
public:
    /**
     * Struct defining a timer, its clock enable register & bit, and its clock
     * input frequency
     * Example:
     *
     *  PWM::Timer t {
     *   TIM4,  // TIM4
     *   &(RCC->APB1ENR),  // APB1 Enable register (TIM4 is on APB1 bus)
     *   RCC_APB1ENR_TIM4EN, // TIM4 enable bit on the APB1 enable register
     *
     *   // APB1 Clock speed
     *   TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)
     *   };
     */
    struct Timer
    {
        TIM_TypeDef* TIM;               // The timer we want to use
        volatile uint32_t* bus_en_reg;  // Pointer to APB1 or APB2 Peripheral
                                        // clock enable register
        uint32_t TIM_EN;  // Enable bit mask for the clock enable register

        unsigned int input_clock_freq;  // Timer input clock frequency [Hz]
    };

    /**
     * @brief Construct a new PWM object
     *
     * @param timer Timer object
     * @param frequency PWM frequency
     * @param duty_cycle_resolution Number of possible values in the range [0,1]
     */
    PWM(Timer timer, unsigned int frequency,
        unsigned int duty_cycle_resolution = 1024);

    ~PWM();

    /**
     * @brief Set the PWM frequency
     * The frequency is changed once the timer is restarted.
     *
     * @param frequency
     */
    void setFrequency(unsigned int frequency);

    /**
     * @brief Set the Duty Cycle Resolution
     * The duty cycle resolution is effectively changed once the timer is
     * restarted.
     * @param duty_cycle_resolution Number of possible values of duty cycle
     */
    void setDutyCycleResolution(unsigned int duty_cycle_resolution);

    /**
     * @brief Enable output on the specified channel
     *
     * @param channel Output channel (1 to 4, refer to datasheet)
     * @param duty_cycle Duty cycle
     * @param mode PWM mode
     * @param polarity Output polarity
     */
    void enableChannel(PWMChannel channel, float duty_cycle,
                       PWMMode mode         = PWMMode::MODE_1,
                       PWMPolarity polarity = PWMPolarity::ACTIVE_HIGH);

    /**
     * @brief Set the duty cycle for the specified channel
     *
     * @param channel
     * @param duty_cycle
     */
    void setDutyCycle(PWMChannel channel, float duty_cycle);

    /**
     * @brief Disables output on the specified channel
     *
     * @param channel Channel to disable(1 to 4, refer to datasheet)
     */
    void disableChannel(PWMChannel channel);

    /**
     * @brief Starts the PWM Generation
     */
    void start();

    /**
     * @brief Stops PWM generation
     */
    void stop();

    /**
     * @brief Returns true if the pwm generator is active
     */
    bool isStarted() { return started; }

    PWMChannelConfig getChannelConfig(PWMChannel channel)
    {
        return channels[static_cast<int>(channel)];
    }

private:
    void hardwareEnableChannel(PWMChannel channel);
    void hardwareDisableChannel(PWMChannel channel);

    void hardwareSetDutyCycle(PWMChannel channel);

    void hardwareUpdateRegisters();

    const Timer timer;
    bool started = false;

    unsigned int frequency;
    unsigned int duty_cycle_resolution;

    PWMChannelConfig channels[4]{};
};

}  // namespace Boardcore
