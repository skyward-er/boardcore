/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <drivers/spi/SPIDriver.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;

GpioPin sckPin  = GpioPin(GPIOE_BASE, 4);
GpioPin misoPin = GpioPin(GPIOE_BASE, 2);
GpioPin mosiPin = GpioPin(GPIOE_BASE, 5);
GpioPin csPin   = GpioPin(GPIOE_BASE, 6);

GpioPin timerCsPin  = GpioPin(GPIOA_BASE, 11);
GpioPin timerSckPin = GpioPin(GPIOB_BASE, 1);

int main()
{
    // Enable clock for SPI4 interface
    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;

    // Enable clock for timers TIM1 and TIM3
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Setup gpio pins
    csPin.mode(Mode::ALTERNATE);
    csPin.alternateFunction(5);
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);
    mosiPin.mode(Mode::ALTERNATE);
    mosiPin.alternateFunction(5);

    timerCsPin.mode(Mode::ALTERNATE);
    timerCsPin.alternateFunction(1);
    timerSckPin.mode(Mode::ALTERNATE);
    timerSckPin.alternateFunction(2);

    // Setup spi as a slave
    SPISignalGenerator spiSignalGenerator{2,
                                          100,
                                          1000000,
                                          SPI::Mode::MODE_0,
                                          GP16bitTimer::Channel::CHANNEL_1,
                                          GP16bitTimer::Channel::CHANNEL_4,
                                          GP16bitTimer::Channel::CHANNEL_4};
    SPISlaveBus bus(SPI4, spiSignalGenerator);
    SPISlave spiSlave(bus, csPin, {});
    spiSlave.config.clockDivider = SPI::ClockDivider::DIV_64;

    SPITransaction transaction(spiSlave);

    uint8_t buffer8[6];
    uint16_t buffer16[6];

    transaction.read();
    delayMs(1);
    transaction.read16();
    delayMs(1);
    transaction.read(buffer8, sizeof(buffer8));
    delayMs(1);
    transaction.read(buffer16, sizeof(buffer16));
    delayMs(1);
    transaction.write((uint8_t)0xAB);
    delayMs(1);
    transaction.write((uint16_t)0xABCD);
    delayMs(1);
    buffer8[0] = 0x01;
    buffer8[1] = 0x23;
    buffer8[2] = 0x45;
    buffer8[3] = 0x67;
    buffer8[4] = 0x89;
    buffer8[5] = 0xAB;
    transaction.write(buffer8, sizeof(buffer8));
    delayMs(1);
    buffer16[0] = 0x0101;
    buffer16[1] = 0x2323;
    buffer16[2] = 0x4545;
    buffer16[3] = 0x6767;
    buffer16[4] = 0x8989;
    buffer16[5] = 0xABAB;
    transaction.write(buffer16, sizeof(buffer16));
    delayMs(1);
    transaction.transfer((uint8_t)0xAB);
    delayMs(1);
    transaction.transfer((uint16_t)0xABCD);
    delayMs(1);
    transaction.transfer(buffer8, sizeof(buffer8));
    delayMs(1);
    transaction.transfer(buffer16, sizeof(buffer16));

    while (true)
        delayMs(1000);
}
