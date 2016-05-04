/* Canbus Test ROM
 * 
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
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
#include <BusTemplate.h>
#include <canbus/CanManager.h>
#include <canbus/CanSocket.h>
#include <canbus/CanUtils.h>

using namespace miosix;

#define DO_ENDLESS_CANBUS_TEST 
#ifndef BOARDNAME
    #define BOARDNAME "UNKNOWN"
#endif

static const uint8_t CAN_MYID = 0x49;

void banner() {
    printf("****** CAN BUS TEST ROM ******\n");
    printf("My ID: '%s'\n", BOARDNAME); 
    printf("\n");
}

void *test_canbus_recv(void *arg) {
    printf("[CAN RECV] Thread started\n");

    CanBus *bus = static_cast<CanBus *>(arg);
    CanSocket socket(CAN_MYID);
    char buf[16]={0};

    printf("[CAN RECV] Opening socket\n");
    socket.open(bus);

    printf("[CAN RECV] Waiting for packets\n");
    while(true) {
        memset(buf, 0, sizeof(buf));
        socket.receive(buf, 16);
        printf("[CAN RECV]: %s\n", buf);
    }
    socket.close();
    return NULL;
}

void test_canbus_send(CanBus *bus) {
    printf("[CAN SEND] Starting test...\n");
    while(true) {
        getchar();
        leds::led0::high();
        const char *pkt = BOARDNAME;
        bus->send(CAN_MYID, (const uint8_t *)pkt, strlen(pkt));
        Thread::sleep(250);
        leds::led0::low();
    }
}

int main() {
    CanManager c(CAN1);

    banner();

    canbus_init_t st0 = {
        CAN1, Mode::ALTERNATE,  9, {CAN1_RX0_IRQn,CAN1_RX1_IRQn}
    };
    c.addBus<GPIOA_BASE, 11, 12>(st0);
    canbus_init_t st1 = {
        CAN2, Mode::ALTERNATE,  9, {CAN2_RX0_IRQn,CAN2_RX1_IRQn}
    };
    c.addBus<GPIOB_BASE, 12, 13>(st2);

    //Receive on CAN1
    Thread::create(test_canbus_recv, 1024, 1, 
            static_cast<void *>(c.getBus(0)), Thread::JOINABLE);
    //Send on CAN2
    test_canbus_send(c.getBus(1));
    
    return 0;
}
