/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <actuators/stepper/StepperPWM.h>
#include <drivers/timer/CountedPWM.h>
#include <miosix.h>

#include <thread>

using namespace miosix;
using namespace Boardcore;

GpioPin directionPin = GpioPin(GPIOC_BASE, 12);
GpioPin enablePin    = GpioPin(GPIOC_BASE, 10);

GpioPin stepPin = GpioPin(GPIOB_BASE, 4);
GpioPin ms1Pin  = GpioPin(GPIOE_BASE, 6);
GpioPin ms2Pin  = GpioPin(GPIOE_BASE, 2);
GpioPin ms3Pin  = GpioPin(GPIOE_BASE, 4);

void setMicroStepping(StepperPWM& stepper, int16_t microStep)
{
    switch (microStep)
    {
        case 1:
            ms1Pin.low();
            ms2Pin.low();
            ms3Pin.low();
            break;
        case 2:
            ms1Pin.high();
            ms2Pin.low();
            ms3Pin.low();
            break;
        case 4:
            ms1Pin.low();
            ms2Pin.high();
            ms3Pin.low();
            break;
        case 8:
            ms1Pin.high();
            ms2Pin.high();
            ms3Pin.low();
            break;
        case 16:
            ms1Pin.high();
            ms2Pin.high();
            ms3Pin.high();
            break;
        default:
            printf("Microsteps value not available!\n");
            return;
    }
    stepper.setMicroStepping(microStep);
}

void doRoutine(StepperPWM& stepper)
{
    stepper.enable();

    for (int i = 1; i < 5; i++)
    {
        stepper.setSpeed(i);

        stepper.setPositionDeg(180);
        delayMs(2 * 1000);
        stepper.setPositionDeg(90);
        delayMs(2 * 1000);
        stepper.setPositionDeg(180);
        delayMs(2 * 1000);
        stepper.setPositionDeg(90);
        delayMs(2 * 1000);
        stepper.setPosition(0);
        delayMs(2 * 1000);
        stepper.moveDeg(-180);
        delayMs(2 * 1000);
        stepper.moveDeg(-360);
        delayMs(2 * 1000);
        stepper.setPosition(0);
        delayMs(5 * 1000);
    }

    stepper.disable();
}

int main()
{
    // Configure stepper motor pins
    directionPin.mode(Mode::OUTPUT);
    enablePin.mode(Mode::OUTPUT);
    ms3Pin.mode(Mode::OUTPUT);
    ms2Pin.mode(Mode::OUTPUT);
    ms1Pin.mode(Mode::OUTPUT);
    stepPin.mode(Mode::ALTERNATE);
    stepPin.alternateFunction(2);

    CountedPWM pwm(
        TIM3, TimerUtils::Channel::CHANNEL_1, TimerUtils::TriggerSource::ITR3,
        TIM4, TimerUtils::Channel::CHANNEL_1, TimerUtils::TriggerSource::ITR2);

    StepperPWM stepper(pwm, stepPin, directionPin, 1, 1.8, false, 8,
                       Stepper::PinConfiguration::COMMON_CATHODE, enablePin);
    setMicroStepping(stepper, 8);

    printf("The stepper is now disabled, waiting 2 seconds...\n");
    delayMs(2 * 1000);

    std::thread th(
        [&stepper]()
        {
            while (true)
            {
                printf("Current position: %.2f\n",
                       stepper.getCurrentDegPosition());
                Thread::sleep(100);
            }
        });
    th.detach();

    while (true)
    {
        printf("Press something to start\n");
        scanf("%*s");

        doRoutine(stepper);
        Thread::sleep(10 * 1000);
    }
}