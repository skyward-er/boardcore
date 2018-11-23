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

class Pwm
{
public:
	/* Freq is intended being expressed in Hz and not KHz.
	 * Duty's range is between 0 and 1 (percentage of one period).
	*/
	Pwm(unsigned int freq, float duty);

	/*It is possible using 2 different channels and two different modes (channel 1 and 2
	 * and mode 1 and 2).
	 * Accepted channel's values: {1,2}.
	 * Accepted mode's values: {1,2}.
	*/
	void configure(int channel, int mode);

	/*Start must be invoked only after configure() has being called.
	 */
	void start();
	void stop();

private:
	unsigned int freq;
	float duty;
};

#endif /* SRC_SHARED_DRIVERS_PWM_PWM_H_ */
