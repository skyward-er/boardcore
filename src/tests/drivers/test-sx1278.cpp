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

#include <cstring>
#include <thread>

/*
Connection diagram:
sx1278[0]:nss  -> stm32:pa1
sx1278[0]:mosi -> stm32:pc12 (SPI3_MOSI)
sx1278[0]:miso -> stm32:pc11 (SPI3_MISO)
sx1278[0]:sck  -> stm32:pc10 (SPI3_SCK)

sx1278[1]:nss  -> stm32:pa2
sx1278[1]:mosi -> stm32:pb15 (SPI2_MOSI)
sx1278[1]:miso -> stm32:pb14 (SPI2_MISO)
sx1278[1]:sck  -> stm32:pb13 (SPI2_SCK)
*/

SPIBus bus1(SPI3);
SPIBus bus2(SPI2);

GpioPin sck1(GPIOC_BASE, 10);
GpioPin miso1(GPIOC_BASE, 11);
GpioPin mosi1(GPIOC_BASE, 12);
GpioPin cs1(GPIOA_BASE, 1);

GpioPin sck2(GPIOB_BASE, 13);
GpioPin miso2(GPIOB_BASE, 14);
GpioPin mosi2(GPIOB_BASE, 15);
GpioPin cs2(GPIOA_BASE, 2);

/// Initialize stm32f407g board
void initBoard()
{
    miosix::FastInterruptDisableLock dLock;

    // Enable SPI1 and SPI2
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_SPI3EN;
    RCC_SYNC();

    // Setup SPI pins
    sck1.mode(miosix::Mode::ALTERNATE);
    sck1.alternateFunction(6);
    miso1.mode(miosix::Mode::ALTERNATE);
    miso1.alternateFunction(6);
    mosi1.mode(miosix::Mode::ALTERNATE);
    mosi1.alternateFunction(6);

    sck2.mode(miosix::Mode::ALTERNATE);
    sck2.alternateFunction(5);
    miso2.mode(miosix::Mode::ALTERNATE);
    miso2.alternateFunction(5);
    mosi2.mode(miosix::Mode::ALTERNATE);
    mosi2.alternateFunction(5);

    cs1.mode(miosix::Mode::OUTPUT);
    cs2.mode(miosix::Mode::OUTPUT);
}

int main()
{

    initBoard();
    cs1.high();
    cs2.high();

    SX1278 sx1278[2] = {
        SX1278(bus1, cs1),
        SX1278(bus2, cs2),
    };

    // Run default configuration
    SX1278::Config config;

    // Configure both devices
    for (int i = 0; i < 2; i++)
    {
        TRACE("Configuring sx1278[%d]...\n", i);
        sx1278[i].init(config);

        auto ver = sx1278[i].getVersion();
        TRACE("Version: %x\n", ver);

        sx1278[i].debugDumpRegisters();
    }

    // miosix::Thread::sleep(5000);

    std::thread recv(
        [&sx1278]()
        {
            int last_recv_message = 0;
            int recv_count        = 0;
            int last_div          = 0;
            while (1)
            {

                uint8_t len = sx1278[0].recv((uint8_t*)&last_recv_message);
                recv_count++;

                if ((last_recv_message / 100) != last_div)
                {
                    TRACE("Packet loss: %.2f%% (%d sent, %d recv)\n",
                          (1.0f -
                           ((float)recv_count / (float)last_recv_message)) *
                              100.0f,
                          last_recv_message, recv_count);
                    last_div = last_recv_message / 100;
                }
            }
        });

    int last_sent_message = 0;
    while (1)
    {
        // TRACE("Trying to send message...\n");

        sx1278[1].send((uint8_t*)&last_sent_message, 4);
        last_sent_message++;

        if (last_sent_message % 100 == 0)
        {
            TRACE("Sent %d messages!\n", last_sent_message);
        }

        // TRACE("Message sent!\n");

        miosix::Thread::sleep(10);
    }

    return 0;
}