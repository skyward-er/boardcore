/* Copyright (c) 2017-2018 Skyward Experimental Rocketry
 * Authors: Andrea Palumbo
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

#ifndef SRC_SHARED_DRIVERS_PWM_PWM_H_
#define SRC_SHARED_DRIVERS_PWM_PWM_H_

#include <miosix.h>

/**
 * @brief Class to generate PWM using hardware timers.
 */
class PWM
{
public:
    struct Timer
    {
        TIM_TypeDef* TIM;      // The timer we want to use
        volatile uint32_t* bus_en_reg;  // APB1 or APB2 Peripheral clock enable register
        uint32_t TIM_EN;       // Enable bit for the clock enable register

        unsigned int input_clock_freq;  // Timer input clock frequency [Hz]
    };

    /**
     * @brief Construct a new PWM object
     *
     * @param timer Struct containing timer register data
     */
    PWM(Timer timer);

    /**
     * @brief Enable the timer in PWM mode
     *
     * @param channel Channel for the selected timer
     * @param duty_cycle_resolution Resolution for the duty cycle
     */

	/**
	 * @brief Enable the timer in PWM mode
	 * 
	 * @param channel Channel for the selected timer
	 * @param frequency PWM frequency
	 * @param enable_compl_out Enable complementary output for supported timers
	 * @param duty_cycle_resolution Resolution for the duty cycle
	 */
    void enable(int channel, unsigned int frequency,
                bool enable_compl_out          = false,
                unsigned int duty_cycle_resolution = 1024);

    /**
     * @brief Set the duty cycle. Can be used when the PWM is already started
     *
     * @param duty_cycle Duty Cycle in fraction of the period. [0,1]
     */
    void setDutyCycle(float duty_cycle);

    /**
     * @brief Starts the PWM Generation
     */
    void start();

    /**
     * @brief Stops PWM generation
     *
     */
    void stop();

    /**
     * @brief Disable the timer
     *
     */
    void disable();

private:
    const Timer timer;

	

	bool enabled = false;
	bool started = false;

	int channel = 0;
    float duty_cycle = 0.5;
	bool enable_compl_out = false;
    uint16_t duty_cycle_res = 1024;
};

#endif /* SRC_SHARED_DRIVERS_PWM_PWM_H_ */
