/* CAN-Bus Driver
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci
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

#ifndef CANBUS_H
#define CANBUS_H

#include <Common.h>
#include "CanUtils.h"

using namespace miosix;
using std::set;

class CanSocket;
class CanManager;

class CanBus {
    public:
        Queue<CanMsg,6> messageQueue;

        bool registerSocket(CanSocket *socket);
        bool unregisterSocket(CanSocket *socket);

        bool send(uint16_t id, const uint8_t *message, uint8_t len);
        void dispatchMessage(CanMsg message);

        void queueHandler();
        volatile CAN_TypeDef* getBus() { return CANx; }

        CanBus(CAN_TypeDef* bus, CanManager* manager, const int id);
        ~CanBus() { }

        CanBus(const CanBus&)            = delete;
        CanBus(const CanBus&&)           = delete;
        CanBus& operator=(const CanBus&) = delete;
    private:
        void canSetup();
        volatile CAN_TypeDef* CANx;
        CanManager* manager;
        const int id;

        FastMutex mutex;
        set<CanSocket *> socket_map[1 << 11];

        volatile bool terminate;
        pthread_t t;

        static void *threadLauncher(void *arg);
};

#endif /* CANBUS_H */
