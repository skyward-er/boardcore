/* CAN-Bus Driver
 *
 * Copyright (c) 2015 Skyward Experimental Rocketry
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

#ifndef CANSOCKET_H
#define CANSOCKET_H

#include <Common.h>
#include "CanBus.h"

using std::list;
using std::pair;

class CanBus;
class CanSocket
{
    public:
        CanSocket(uint16_t filter_id);
        void open(CanBus *bus);

        bool receive(void *message, int& size);

        void close();

        bool isOpen() const { return bus != NULL; }

        void addToMessageList(unsigned char *message, uint8_t size);
        uint16_t getFilterId() const { return filter_id; }

        CanSocket& operator=(const CanSocket&)=delete;
        ~CanSocket();

    private:
        CanBus *bus = NULL;
        const uint16_t filter_id;

        pthread_mutex_t mutex;
        pthread_cond_t cond;

        typedef pair<const unsigned char *, int> msg_p;
        list<msg_p> receivedMessageQueue;
};


#endif /* CANSOCKET_H */
