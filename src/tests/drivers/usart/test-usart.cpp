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
#include "utils/SerialInterface.h"

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
USARTInterface::Baudrate baudrates[] = {
    USARTInterface::Baudrate::B2400,   USARTInterface::Baudrate::B9600,
    USARTInterface::Baudrate::B19200,  USARTInterface::Baudrate::B38400,
    USARTInterface::Baudrate::B57600,  USARTInterface::Baudrate::B115200,
    USARTInterface::Baudrate::B230400, USARTInterface::Baudrate::B460800,
    USARTInterface::Baudrate::B921600};

/**
 * Communication: src -> dst
 * tests the writeString, write and read methods of the USART drivers
 */
bool testCommunicationSequential(USARTInterface *src, USARTInterface *dst)
{
    char buf_rx[64];
    StructToSend struct_rx;
    bool passed = true;

    /************************** SENDING STRING **************************/
    printf("Sending string\n");

    printf("\t%d--> sent: \t'%s'\n", src->getId(), buf_tx);
    src->writeString(buf_tx);
    // Thread::sleep(10); // enable to pass the test with STM32SerialWrapper
    dst->read(buf_rx, 64);

    printf("\t%d<-- received: \t'%s'\n", dst->getId(), buf_rx);

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
    dst->read(&struct_rx, sizeof(StructToSend));
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

    return passed;
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
    bool testPassed = true;
    printf("*** SERIAL 3 WORKING!\n");
    for (unsigned int iBaud = 0;
         iBaud < sizeof(baudrates) / sizeof(baudrates[0]); iBaud++)
    {
        USARTInterface::Baudrate baudrate = baudrates[iBaud];
        printf("\n\n########################### %d\n", (int)baudrate);

        // declaring the usart peripherals
        STM32SerialWrapper usartx(USART1, baudrate, u1rx2::getPin(),
                                  u1tx1::getPin());
        usartx.init();

        USART usarty(UART4, baudrate);
        // usarty.initPins(u5tx::getPin(), 8, u5rx::getPin(), 8);
        // usarty.setOversampling(false);
        // usarty.setStopBits(1);
        // usarty.setWordLength(USART::WordLength::BIT8);
        // usarty.setParity(USART::ParityBit::NO_PARITY);
        usarty.init();

        // testing transmission (both char and binary) "serial 1 <- serial 2"
        testPassed &= testCommunicationSequential(&usartx, &usarty);

        // testing transmission (both char and binary) "serial 1 -> serial 2"
        testPassed &= testCommunicationSequential(&usarty, &usartx);
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
    return 0;
}
