/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Matteo Michele Piazzolla
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Common.h>
#include <drivers/canbus/CanManager.h>
#include <drivers/canbus/CanUtils.h>

using namespace std;
using namespace miosix;

#define CAN_PACKETID 0x49

void handleCan (CanMsg message) {
    unsigned char buf[65] = {0};
    memcpy(buf, message.Data, message.DLC);
    printf("Received %s\n", buf);
}

int main()
{
    CanManager c(CAN1);

    canbus_init_t st = {
        CAN1, Mode::ALTERNATE, 9, {CAN1_RX0_IRQn, CAN1_RX1_IRQn}};
        
    c.addBus<GPIOA_BASE, 11, 12>(st, handleCan);
    // canbus_init_t st2= {
    //    CAN2, Mode::ALTERNATE,  9, {CAN2_RX0_IRQn,CAN2_RX1_IRQn}
    //};
    // c.addBus<GPIOB_BASE, 5, 6>(st2);

    CanBus *bus = c.getBus(0);
    c.addHWFilter(CAN_PACKETID, 0);

    printf("*** Ready ***\n");

    while (1)
    {
        ledOn();
        const char *pkt = "TestMSG";
        bus->send(CAN_PACKETID, (const uint8_t *)pkt, strlen(pkt));
        //socket.receive(buf, 64);
        Thread::sleep(250);
        ledOff();
        Thread::sleep(150);
    }

}
