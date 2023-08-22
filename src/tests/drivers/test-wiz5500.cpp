/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <drivers/WIZ5500/WIZ5500.h>

using namespace Boardcore;
using namespace miosix;

#if defined _BOARD_STM32F429ZI_STM32F4DISCOVERY

using cs = Gpio<GPIOC_BASE, 13>;
using sck = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;

SPIBus bus(SPI4);

#else
#error "Target not supported"
#endif

void setupBoard() {
    sck::mode(Mode::ALTERNATE);
    sck::alternateFunction(5);
    miso::mode(Mode::ALTERNATE);
    miso::alternateFunction(5);
    mosi::mode(Mode::ALTERNATE);
    mosi::alternateFunction(5);
    cs::mode(Mode::OUTPUT);
    cs::high();
}

int main() {
    setupBoard();
    
    WizCore wiz = WizCore(bus, cs::getPin(), SPI::ClockDivider::DIV_64);


    uint8_t version = wiz.spiRead8(0x00, 0x0039);
    printf("Version: %d\n", version);

    uint16_t rtr = wiz.spiRead16(0x00, 0x0019);
    printf("RTR: %x\n", rtr);

    while(1);
    return 0;
}