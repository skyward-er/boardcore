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

#include <cassert>

#include "drivers/usart/USART.h"
#include "miosix.h"
#include "string.h"
#include "thread"

using namespace miosix;
using namespace Boardcore;

typedef struct
{
    char dataChar;
    int dataInt;
    float dataFloat;
    double dataDouble;
} StructToSend;
StructToSend struct_tx       = {'C', 42, 420.69, 48.84};
char buf_tx[64]              = "Testing communication :D";
USART::Baudrate baudrates[4] = {
    USART::Baudrate::B2400,
    USART::Baudrate::B9600,
    USART::Baudrate::B19200,
    USART::Baudrate::B115200,
};

// function for the thread that has to read from serial
void readSer(USART *s, void *rcv)
{
    s->read(rcv, 64);
    // printf("\t<--%d received: \t'%s'\n", s->getId(), rcv);
}

// Communicatio: src -> dst
void testCommunicationSequential(USART *src, USART *dst)
{
    char buf_rx[64];
    StructToSend struct_rx;

    // SENDING STRING
    printf("Sending string\n");
    printf("\t-->%d sending: \t'%s'\n", src->getId(), buf_tx);
    src->writeString(buf_tx);
    Thread::sleep(100);
    dst->read(buf_rx, 64);
    printf("\t<--%d received: \t'%s'\n", dst->getId(), buf_rx);

    if (strcmp(buf_tx, buf_rx) == 0)
    {
        printf("*** %d -> %d WORKING!\n", src->getId(), dst->getId());
    }
    else
    {
        printf("### %d -> %d ERROR!\n", src->getId(), dst->getId());
    }

    // SENDING BINARY DATA
    printf("Sending binary data\n");
    printf("\t-->%d sending: \t'%c,%d,%f,%f'\n", src->getId(),
           struct_tx.dataChar, struct_tx.dataInt, struct_tx.dataFloat,
           struct_tx.dataDouble);
    src->write(&struct_tx, sizeof(StructToSend));
    Thread::sleep(100);
    dst->read(&struct_rx, sizeof(StructToSend));
    printf("\t<--%d received: \t'%c,%d,%f,%f'\n", dst->getId(),
           struct_rx.dataChar, struct_rx.dataInt, struct_rx.dataFloat,
           struct_rx.dataDouble);

    if (memcmp(&struct_tx, &struct_rx, sizeof(StructToSend)) == 0)
    {
        printf("*** %d -> %d WORKING!\n", src->getId(), dst->getId());
    }
    else
    {
        printf("### %d -> %d ERROR!\n", src->getId(), dst->getId());
    }
}

//##############################################################################//
/*
 * // USART1: tx=PA9  rx=PA10 cts=PA11 rts=PA12
 * USART1: tx=PB6  rx=PB7
 * USART2: tx=PA2  rx=PA3  cts=PA0  rts=PA1
 * USART3: tx=PB10 rx=PB11 cts=PB13 rts=PB14
 */
int main()
{
    printf("*** SERIAL 3 WORKING!\n");
    for (int iBaud = 0; iBaud < 4; iBaud++)
    {
        // Setting the baudrate to 2400, maximum functioning baudrate for the
        // Max485 adapters
        Boardcore::USART usart1(USART1, baudrates[iBaud]);
        usart1.setOversampling(false);
        usart1.setStopBits(1);
        usart1.setWordLength(USART::WordLength::BIT8);
        usart1.setParity(USART::ParityBit::NO_PARITY);
        usart1.init();

        Boardcore::USART usart2(USART2, baudrates[iBaud]);
        usart2.setOversampling(false);
        usart2.setStopBits(1);
        usart2.setWordLength(USART::WordLength::BIT8);
        usart2.setParity(USART::ParityBit::NO_PARITY);
        usart2.init();

        printf("\n\n########################### %d\n", baudrates[iBaud]);
        // testing transmission "serial 1 <- serial 2"
        testCommunicationSequential(&usart1, &usart2);

        // testing transmission "serial 1 -> serial 2"
        testCommunicationSequential(&usart2, &usart1);

        Thread::sleep(1000);
    }

    return 0;
}
