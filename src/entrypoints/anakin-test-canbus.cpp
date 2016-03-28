/* Canbus Test ROM
 * 
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Matteo Piazzolla
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
        const char *pkt = BOARDNAME;
        bus->send(CAN_MYID, (const uint8_t *)pkt, strlen(pkt));
        Thread::sleep(250);
    }
}

int main() {
    CanManager c(CAN1);
    CanBus *bus;

    banner();

    canbus_init_t st = {
        CAN1, Mode::ALTERNATE,  9, {CAN1_RX0_IRQn,CAN1_RX1_IRQn}
    };
    c.addBus<GPIOA_BASE, 11, 12>(st);

    printf("[MAIN] Requesting bus\n");
    bus = c.getBus(0);

    Thread::create(test_canbus_recv, 1024, 1, static_cast<void *>(bus), Thread::JOINABLE);
    test_canbus_send(bus);
    
    return 0;
}
