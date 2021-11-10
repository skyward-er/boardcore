/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <Debug.h>
#include <drivers/SX1278/SX1278.h>
#include <thread>
#include <cstring>

/*
Connection diagram:
sx1278[0]:nss  -> stm32:pa3
sx1278[0]:mosi -> stm32:pa7  (SPI1_MOSI)
sx1278[0]:miso -> stm32:pa6  (SPI1_MISO)
sx1278[0]:sck  -> stm32:pa5  (SPI1_SCK)

sx1278[0]:nss  -> stm32:pa2 
sx1278[0]:mosi -> stm32:pb15 (SPI2_MOSI)
sx1278[0]:miso -> stm32:pb14 (SPI2_MISO)
sx1278[0]:sck  -> stm32:pb13 (SPI2_SCK)
*/

SPIBus bus1(SPI1);
SPIBus bus2(SPI2);

GpioPin sck1(GPIOA_BASE, 5);
GpioPin miso1(GPIOA_BASE, 6);
GpioPin mosi1(GPIOA_BASE, 7);
GpioPin cs1(GPIOA_BASE, 3);

GpioPin sck2(GPIOB_BASE, 13);
GpioPin miso2(GPIOB_BASE, 14);
GpioPin mosi2(GPIOB_BASE, 15);
GpioPin cs2(GPIOA_BASE, 2);

/// Initialize stm32f407g board
void initBoard() {
    miosix::FastInterruptDisableLock dLock;

    // Enable SPI1 and SPI2
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // Setup SPI pins
    sck1.mode(miosix::Mode::ALTERNATE);
    sck1.alternateFunction(5);
    miso1.mode(miosix::Mode::ALTERNATE);
    miso1.alternateFunction(5);
    mosi1.mode(miosix::Mode::ALTERNATE);
    mosi1.alternateFunction(5);

    sck2.mode(miosix::Mode::ALTERNATE);
    sck2.alternateFunction(5);
    miso2.mode(miosix::Mode::ALTERNATE);
    miso2.alternateFunction(5);
    mosi2.mode(miosix::Mode::ALTERNATE);
    mosi2.alternateFunction(5);
    
    cs1.mode(miosix::Mode::OUTPUT);
    cs2.mode(miosix::Mode::OUTPUT);
}

int main() {

    initBoard();
    cs1.high();
    cs2.high();

    SX1278 sx1278[2] = {
        SX1278(bus1, cs1),
        SX1278(bus2, cs2)
    };

    // Configure both devices
    for(int i = 0; i < 2; i++) {
        TRACE("Configuring sx1278[%d]...\n", i);
        sx1278[i].init();

        auto ver = sx1278[i].getVersion();
        TRACE("Version: %x\n", ver);

        // Set bitrate to 4.8kb/s
        sx1278[i].setBitrate(4800.0f);
        // Set frequency deviation to 5kHz
        sx1278[i].setFreqDev(5000);
        // Set carrier frequency to 434MHz
        sx1278[i].setFreqRF(434000000);

        sx1278[i].debugDumpRegisters();
    }

    miosix::Thread::sleep(5000);

    std::thread recv([&sx1278]() {
        while(1) {
            uint8_t buf[256] = {0};
            uint8_t len = sx1278[0].recv(buf);

            TRACE("Received %d data! '%s'\n", len, buf);
        }
    });

    const char *msg = "Test message";
    while(1) {
        uint8_t len = strlen(msg) + 1;
        sx1278[1].send(reinterpret_cast<const uint8_t*>(msg), len);

        TRACE("Message sent!\n");

        miosix::Thread::sleep(500);
    }

    return 0;
}