/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <miosix.h>
#include <utils/PinObserver/ExternalPinObserver.h>

#include <functional>

using namespace Boardcore;
using namespace miosix;
using namespace std;
using namespace std::placeholders;

static constexpr unsigned int POLL_INTERVAL = 20;

static constexpr Boardcore::ExternalGpioPin PINA1(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN1);

static constexpr Boardcore::ExternalGpioPin PINA2(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN2);
static constexpr Boardcore::ExternalGpioPin PINA3(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN3);

static constexpr Boardcore::ExternalGpioPin LED1(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN4);

void onTransition(ExternalGpioPin pin, PinTransition transition)
{
    if (pin.getPort() == PINA1.getPort() && pin.getPin() == PINA1.getPin())
        printf("PINA1 transition: ");
    if (pin.getPort() == PINA2.getPort() && pin.getPin() == PINA2.getPin())
        printf("PINA2 transition: ");
    if (pin.getPort() == PINA3.getPort() && pin.getPin() == PINA3.getPin())
        printf("PINA3 transition: ");

    if (transition == PinTransition::FALLING_EDGE)
        printf("FALLING_EDGE\n");
    else
        printf("RISING_EDGE\n");
}

int main()
{
    SPIBus spi2(SPI2);
    GpioPin sck(GPIOC_BASE, 3);
    sck.mode(Mode::ALTERNATE);
    sck.alternateFunction(5);
    GpioPin miso(GPIOB_BASE, 14);
    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);
    GpioPin mosi(GPIOC_BASE, 3);
    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);
    GpioPin cs(GPIOC_BASE, 1);
    cs.mode(Mode::OUTPUT);
    cs.high();

    auto expander = new MCP23S17(spi2, cs);

    TaskScheduler scheduler;
    ExternalPinObserver observer{scheduler, expander, POLL_INTERVAL};

    if (!expander->init())
    {
        printf("Failed to initialize MCP23S17\n");
        return -1;
    }

    expander->setPinMode(PINA1.getPort(), PINA1.getPin(),
                         MCP23S17Defs::MODE::INPUT_PULL_UP);
    expander->setPinMode(PINA2.getPort(), PINA2.getPin(),
                         MCP23S17Defs::MODE::INPUT);
    expander->setPinMode(PINA3.getPort(), PINA3.getPin(),
                         MCP23S17Defs::MODE::INPUT);

    expander->setPinMode(LED1.getPort(), LED1.getPin(),
                         MCP23S17Defs::MODE::OUTPUT);

    observer.registerPinCallback(
        PINA1, [](PinTransition transition, auto pinData)
        { onTransition(PINA1, transition); }, 10);

    observer.registerPinCallback(
        PINA2, [](PinTransition transition, auto pinData)
        { onTransition(PINA2, transition); }, 10);

    observer.registerPinCallback(
        PINA3, [](PinTransition transition, auto pinData)
        { onTransition(PINA3, transition); }, 10);

    scheduler.start();
    bool ledstate = false;

    while (true)
    {
        bool state = expander->getPinValue(PINA1.getPort(), PINA1.getPin());
        printf("PINA1 state: %s\n", state ? "HIGH" : "LOW");

        expander->setPinValue(LED1.getPort(), LED1.getPin(), ledstate);
        ledstate = !ledstate;
        Thread::sleep(5000);
    }
}
