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

using namespace miosix;
using namespace Boardcore;

/**
setup:
    - short the rx and tx pins of the port you wanto to test
    - connect the default serial port to the pc
*/

int main()
{
    char buf_tx[64] = "Questo e' un test!";

    typedef struct
    {
        char dataChar;
        int dataInt;
        float dataFloat;
        double dataDouble;
    } StructToSend;
    StructToSend struct_tx;
    struct_tx = {'C', 42, 420.69, 48.84};

    /*
     * // USART1: tx=PA9  rx=PA10 cts=PA11 rts=PA12
     * USART1: tx=PB6  rx=PB7
     * USART2: tx=PA2  rx=PA3  cts=PA0  rts=PA1
     * USART3: tx=PB10 rx=PB11 cts=PB13 rts=PB14
     */
    Boardcore::USART usart1(USART1, Boardcore::USART::Baudrate::B19200);
    usart1.setStopBits(1);
    usart1.setWordLength(USART::WordLength::BIT8);
    usart1.setParity(USART::ParityBit::NO_PARITY);
    usart1.init();

    printf("Data to be received: char:%c, int:%d, float:%f, double:%f\n",
           struct_tx.dataChar, struct_tx.dataInt, struct_tx.dataFloat,
           struct_tx.dataDouble);

    while (true)
    {
        // new data to receive
        StructToSend struct_rx;
        char buf_rx[64];

        // testing write function for strings
        usart1.writeString(buf_tx);
        Thread::sleep(100);
        {
            int nRead = usart1.read(buf_rx, 64);
            printf("string: %s\n", buf_rx);
        }

        // testing write function for binary data
        usart1.write(&struct_tx, sizeof(StructToSend));
        Thread::sleep(100);
        {
            int nRead = usart1.read(&struct_rx, sizeof(StructToSend));
            printf("char:   %c\n", struct_rx.dataChar);
            printf("int:    %d\n", struct_rx.dataInt);
            printf("float:  %f\n", struct_rx.dataFloat);
            printf("double: %f\n", struct_rx.dataDouble);
        }

        printf("\n");
        Thread::sleep(2000);
    }

    return 0;
}
