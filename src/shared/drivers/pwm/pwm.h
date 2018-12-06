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
 * Class is NOT thread-safe. Every instante must be used by a single thread.
 */
class PWM
{
public:
    struct Timer
    {
        TIM_TypeDef* TIM;  // The timer we want to use
        volatile uint32_t*
            bus_en_reg;   // APB1 or APB2 Peripheral clock enable register
        uint32_t TIM_EN;  // Enable bit for the clock enable register

        unsigned int input_clock_freq;  // Timer input clock frequency [Hz]
    };

    enum class Channel : int
    {
        CH1,
        CH2,
        CH3,
        CH4
    };

    /**
     * @brief PWM channel output polarity
     *
     */
    enum class Polarity
    {
        ACTIVE_HIGH,
        ACTIVE_LOW
    };

    /**
     * @brief PWM mode selection. Refer to datasheet
     * MODE_1  Channel high when CNT < CCRx
     * MODE_2  Channel high when CNT > CCRx
     */
    enum class Mode
    {
        MODE_1,  // Channel high when CNT < CCRx
        MODE_2   // Channel high when CNT > CCRx
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

    /**
     * @brief Set the PWM frequency
     * The frequency is changed once the timer is restarted.
     *
     * @param frequency
     */
    void setFrequency(unsigned int frequency);

    /**
     * @brief Set the Duty Cycle Resolution
     * The duty cycle resolution is effectively changed once the timer is restarted.
     * @param duty_cycle_resolution Number of possible values of duty cycle
     */
    void setDutyCycleResolution(unsigned int duty_cycle_resolution);

    /**
     * @brief Enable output on the specified channel
     *
     * @param mode PWM mode
     * @param channel Output channel (1 to 4, refer to datasheet)
     * @param polarity Output polarity
     */
    void enableChannel(Channel channel, float duty_cycle, Mode mode = Mode::MODE_1,
                       Polarity polarity = Polarity::ACTIVE_HIGH);

    /**
     * @brief Set the duty cycle for the specified channel
     *
     * @param channel
     * @param duty_cycle
     */
    void setDutyCycle(Channel channel, float duty_cycle);

    /**
     * @brief Disables output on the specified channel
     *
     * @param channel Channel to disable(1 to 4, refer to datasheet)
     */
    void disableChannel(Channel channel);

    /**
     * @brief Starts the PWM Generation
     */
    void start();

    /**
     * @brief Stops PWM generation
     */
    void stop();

private:
    struct ChannelConfig
    {
        bool enabled = false;
        int test = 5;
        float duty_cycle;
        Mode mode;
        Polarity polarity;
    };

   void hardwareEnableChannel(Channel channel);
   void hardwareDisableChannel(Channel channel);

   void hardawreSetDutyCycle(Channel channel);


    const Timer timer;
    bool started = false;

    unsigned int frequency;
    unsigned int duty_cycle_resolution;

    ChannelConfig channels[4]{};
};

#endif /* SRC_SHARED_DRIVERS_PWM_PWM_H_ */
