/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <fmt/format.h>

#include <cassert>

#include "drivers/usart/USART.h"
#include "miosix.h"
#include "string"
#include "string.h"
#include "thread"

using namespace miosix;
using namespace Boardcore;

/**
 * SETUP:
 * - connect the default serial (USART3) to the pc
 * - connect usartx_rx to the usarty_tx
 * - connect usarty_rx to the usartx_tx
 *
 * WARNING: If using the STM32SerialWrapper to read, the test passes only if we:
 *  1. do a write/writeString with USART or STM32SerialWrapper;
 *  2. sleep 10ms or more (or a printf...)
 *  3. we finally read from STM32SerialWrapper::read method.
 * if we omit the waiting time we end up failing the test. If more data is sent,
 * then more baudrates tests will fail (from 2400 to greater). The USART driver
 * doesn't have this problem.
 */

// A nice feature of the stm32 is that the USART are connected to the same
// GPIOS in all families, stm32f1, f2, f4 and l1. Additionally, USART1 and
// USART6 are always connected to the APB2, while the other USART/UARTs are
// connected to the APB1.

// USART1: AF7
typedef miosix::Gpio<GPIOB_BASE, 6> u1tx1;
typedef miosix::Gpio<GPIOB_BASE, 7> u1rx1;
typedef miosix::Gpio<GPIOA_BASE, 9> u1tx2;
typedef miosix::Gpio<GPIOA_BASE, 10> u1rx2;
// typedef miosix::Gpio<GPIOA_BASE, 11> u1cts;
// typedef miosix::Gpio<GPIOA_BASE, 12> u1rts;

// USART2: AF7
typedef miosix::Gpio<GPIOA_BASE, 2> u2tx1;
typedef miosix::Gpio<GPIOA_BASE, 3> u2rx1;
typedef miosix::Gpio<GPIOD_BASE, 5> u2tx2;
typedef miosix::Gpio<GPIOD_BASE, 6> u2rx2;
// typedef miosix::Gpio<GPIOA_BASE, 0> u2cts;
// typedef miosix::Gpio<GPIOA_BASE, 1> u2rts;

// USART3: AF7
typedef miosix::Gpio<GPIOB_BASE, 10> u3tx1;
typedef miosix::Gpio<GPIOB_BASE, 11> u3rx1;
typedef miosix::Gpio<GPIOD_BASE, 8> u3tx2;
typedef miosix::Gpio<GPIOD_BASE, 9> u3rx2;
// typedef miosix::Gpio<GPIOB_BASE, 13> u3cts;
// typedef miosix::Gpio<GPIOB_BASE, 14> u3rts;

// UART4: AF8
typedef miosix::Gpio<GPIOA_BASE, 0> u4tx1;
typedef miosix::Gpio<GPIOA_BASE, 1> u4rx1;
typedef miosix::Gpio<GPIOC_BASE, 10> u4tx2;
typedef miosix::Gpio<GPIOC_BASE, 11> u4rx2;

// UART5: AF8
typedef miosix::Gpio<GPIOC_BASE, 12> u5tx;
typedef miosix::Gpio<GPIOD_BASE, 2> u5rx;

// USART6: AF8
typedef miosix::Gpio<GPIOC_BASE, 6> u6tx1;
typedef miosix::Gpio<GPIOC_BASE, 7> u6rx1;
#ifdef STM32F429xx
typedef miosix::Gpio<GPIOG_BASE, 14> u6tx2;
typedef miosix::Gpio<GPIOG_BASE, 9> u6rx2;

// USART7: AF8
typedef miosix::Gpio<GPIOE_BASE, 8> u7tx1;
typedef miosix::Gpio<GPIOE_BASE, 7> u7rx1;
typedef miosix::Gpio<GPIOF_BASE, 7> u7tx2;
typedef miosix::Gpio<GPIOF_BASE, 6> u7rx2;

// USART8: AF8
typedef miosix::Gpio<GPIOE_BASE, 1> u8tx;
typedef miosix::Gpio<GPIOE_BASE, 0> u8rx;
#endif  // STM32F429xx

typedef struct
{
    char dataChar;
    int dataInt;
    float dataFloat;
    double dataDouble;

    std::string print()
    {
        return fmt::format("{},{:d},{:f},{:f}", dataChar, dataInt, dataFloat,
                           dataDouble);
    }
} StructToSend;
StructToSend struct_tx = {'C', 42, 420.69, 48.84};
char buf_tx[64]        = "Testing communication, but very very very loong :D";
int baudrates[]        = {2400,   9600,   19200,  38400,  57600,
                          115200, 230400, 256000, 460800, 921600};

/**
 * Communication: src -> dst
 * tests the writeString, write and read methods of the USART drivers
 */
bool testCommunicationSequential(USARTInterface *src, USARTInterface *dst)
{
    char buf_rx[64] = {0};
    StructToSend struct_rx{0};
    size_t nReads{0};
    bool passed = true;

    /************************** SENDING STRING **************************/
    printf("Sending string\n");

    printf("\t%d--> sent: \t'%s'\n", src->getId(), buf_tx);
    src->writeString(buf_tx);
    // Thread::sleep(10); // enable to pass the test with STM32SerialWrapper
    if (!dst->readBlocking(buf_rx, 64, nReads))
    {
        printf("### NO DATA READ ###\n");
        passed = false;
    }

    printf("\t%d<-- received: \t'%s'\n", dst->getId(), buf_rx);

    if (nReads != strlen(buf_tx) + 1)
    {
        printf("### READ WRONG NUMBER OF BYTES ###\n");
        passed = false;
    }
    else
    {
        printf("*** READ EXACT NUMBER OF BYTES ***\n");
    }

    if (strcmp(buf_tx, buf_rx) == 0)
    {
        printf("*** %d -> %d WORKING!\n", src->getId(), dst->getId());
    }
    else
    {
        printf("### %d -> %d ERROR!\n", src->getId(), dst->getId());
        passed = false;
    }

    /*********************** SENDING BINARY DATA ************************/

    printf("Sending binary data\n");
    printf("\t%d--> sent: \t'%s'\n", src->getId(), struct_tx.print().c_str());
    src->write(&struct_tx, sizeof(StructToSend));
    // Thread::sleep(10); // enable to pass the test with STM32SerialWrapper
    if (!dst->readBlocking(&struct_rx, sizeof(StructToSend)))
    {
        printf("### NO DATA READ ###\n");
        passed = false;
    }

    printf("\t%d<-- received: \t'%s'\n", dst->getId(),
           struct_rx.print().c_str());

    if (memcmp(&struct_tx, &struct_rx, sizeof(StructToSend)) == 0)
    {
        printf("*** %d -> %d WORKING!\n", src->getId(), dst->getId());
    }
    else
    {
        printf("### %d -> %d ERROR!\n", src->getId(), dst->getId());
        passed = false;
    }

    // Testing the non blocking read only if USART class
    if ((dynamic_cast<const USART *>(dst) != nullptr) &&
        !dynamic_cast<USART *>(dst)->read(&struct_rx, sizeof(StructToSend)))
    {
        printf("Non blocking read passed!\n");
    }

    return passed;
}

bool testClearQueue(USART *src, USART *dst)
{
    char buf[128];
    size_t nReads{0};
    src->writeString("Transmitting useless stuff!");
    // miosix::delayUs(1000);
    dst->clearQueue();

    // Can be commented to test without read
    if (dst->read(buf, 128, nReads))
    {
        printf("### read something after the clearQueue: %s (%zu bytes)\n", buf,
               nReads);
        // Shouldn't read anything
        return false;
    }

    src->writeString("Now transmitting the juicy stuff :P");
    dst->readBlocking(buf, 128, nReads);

    // After the clearQueue we should only read the things written after
    if (strcmp(buf, "Now transmitting the juicy stuff :P") != 0)
    {
        printf(
            "### read something different than the things sent: %s (%zu "
            "bytes)\n",
            buf, nReads);
        return false;
    }

    printf("*** clearQueue test passed\n");
    return true;
}

/* Available default pins:
 * - USART1: tx=PA9  rx=PA10
 * - USART2: tx=PA2  rx=PA3
 * - USART3: tx=PB10 rx=PB11
 * - UART4:  tx=PA0 rx=PA1
 * - UART5:  tx=PC12 rx=PD2
 * - USART6: tx=PC6 rx=PC7
 * - UART7: tx=PE8 rx=PE7
 * - UART8: tx=PE1 rx=PE0
 */
int main()
{
    // Init serial port pins
    u6rx1::getPin().mode(miosix::Mode::ALTERNATE);
    u6rx1::getPin().alternateFunction(8);
    u6tx1::getPin().mode(miosix::Mode::ALTERNATE);
    u6tx1::getPin().alternateFunction(8);

    u4rx2::getPin().mode(miosix::Mode::ALTERNATE);
    u4rx2::getPin().alternateFunction(8);
    u4tx2::getPin().mode(miosix::Mode::ALTERNATE);
    u4tx2::getPin().alternateFunction(8);

    for (;;)
    {
        bool testPassed = true;
        printf("*** SERIAL 3 WORKING!\n");

        for (int baudrate : baudrates)
        {
            printf("\n\n########################### %d\n", baudrate);
            // declaring the usart peripherals
            USART usartx(USART6, baudrate);
            // usartx.setBaudrate(baudrate);
            // usartx.setOversampling(false);
            // usartx.setStopBits(1);
            // usartx.setWordLength(USART::WordLength::BIT8);
            // usartx.setParity(USART::ParityBit::NO_PARITY);

            USART usarty(UART4, baudrate);
            // STM32SerialWrapper usarty(UART4, baudrate, u4rx2::getPin(),
            //                           u4tx2::getPin());

            // testing transmission (both char and binary) "serial 1 <- serial
            // 2"
            testPassed &= testCommunicationSequential(&usartx, &usarty);
            testPassed &= testClearQueue(&usartx, &usarty);

            // testing transmission (both char and binary) "serial 1 -> serial
            // 2"
            testPassed &= testCommunicationSequential(&usarty, &usartx);
            testPassed &= testClearQueue(&usarty, &usartx);
        }

        if (testPassed)
        {
            printf(
                "********************************\n"
                "***        TEST PASSED       ***\n"
                "********************************\n");
        }
        else
        {
            printf(
                "################################\n"
                "###        TEST FAILED       ###\n"
                "################################\n");
        }
        Thread::sleep(5000);
    }
    return 0;
}
