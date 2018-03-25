/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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

#ifndef TMTCSENDER_H
#define TMTCSENDER_H

#include "TMTCManager.h"
#include "events/SyncQueue.h"

class TMTCSender : ActiveObject {
        
public:
    TMTCSender(Gamma868* gamma){}
    
    void addToBuffer(message_t* message){
	   outBuffer.put(message);
    }

    void printBuffer() {
        message_t* msg;

        while (outBuffer.len() > 0)
        {
            msg = outBuffer.get();
            printf ("%d\n", msg->type);
        }
    }
    
    ~TMTCSender() {}
    
protected:
    void run() {
       printBuffer();
    }
    
private:
    SynchronizedQueue<message_t*> outBuffer;
    Gamma868* gamma;
    
};

#endif