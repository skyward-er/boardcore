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
#include <drivers/interrupt/external_interrupts.h>

#include <cstring>
#include <thread>

/*
Connection diagram:
sx1278[0]:nss  -> stm32:pa1
sx1278[0]:dio0 -> stm32:pc15
sx1278[0]:mosi -> stm32:pc12 (SPI3_MOSI)
sx1278[0]:miso -> stm32:pc11 (SPI3_MISO)
sx1278[0]:sck  -> stm32:pc10 (SPI3_SCK)

sx1278[1]:nss  -> stm32:pa2
sx1278[1]:dio0 -> stm32:pc0
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
GpioPin dio1(GPIOC_BASE, 15);

GpioPin sck2(GPIOB_BASE, 13);
GpioPin miso2(GPIOB_BASE, 14);
GpioPin mosi2(GPIOB_BASE, 15);
GpioPin cs2(GPIOA_BASE, 2);
GpioPin dio2(GPIOC_BASE, 0);

SX1278 *sx1278_rx = nullptr;
SX1278 *sx1278_tx = nullptr;

void __attribute__((used)) EXTI0_IRQHandlerImpl() { sx1278_rx->handleDioIRQ(); }

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
    dio1.mode(miosix::Mode::INPUT);

    cs2.mode(miosix::Mode::OUTPUT);
    dio2.mode(miosix::Mode::INPUT);
}

int main()
{

    initBoard();
    cs1.high();
    cs2.high();
    enableExternalInterrupt(GPIOC_BASE, 0, InterruptTrigger::RISING_EDGE);

    sx1278_rx = new SX1278(bus2, cs2, dio2);
    sx1278_tx = new SX1278(bus1, cs1, dio1);

    // Run default configuration
    SX1278::Config config;

    TRACE("Configuring sx1278_rx...\n");
    config.enable_int = true;
    sx1278_rx->init(config);

    TRACE("Configuring sx1278_tx...\n");
    config.enable_int = false;
    sx1278_tx->init(config);

    // miosix::Thread::sleep(5000);
    const int LOG_INTERVAL = 100;

    std::thread recv(
        []()
        {
            int recv_count        = 0;
            int last_recv_message = 0;
            int last_div          = 0;
            while (1)
            {

                uint8_t len = sx1278_rx->recv((uint8_t *)&last_recv_message);
                recv_count++;

                int new_div = last_recv_message / LOG_INTERVAL;
                if (new_div != last_div)
                {
                    TRACE("Packet loss: %.2f%% (%d sent, %d recv)\n",
                          (1.0f -
                           ((float)recv_count / (float)last_recv_message)) *
                              100.0f,
                          last_recv_message, recv_count);
                    last_div = new_div;
                }
            }
        });

    int last_sent_message = 0;
    while (1)
    {
        last_sent_message++;
        sx1278_tx->send((uint8_t *)&last_sent_message, 4);

        if (last_sent_message % LOG_INTERVAL == 0)
        {
            TRACE("Sent %d messages!\n", last_sent_message);
        }

        miosix::Thread::sleep(5);
    }

    return 0;
}