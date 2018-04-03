/* Copyright (c) 2018 Skyward Experimental Rocketry
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

#include "TMTC_config.h"

/*
 * The Sender class is an ActiveObject that forwards on the RF module 
 * the packets that are in its buffer using the driver's blocking functions.
 */
class Sender : ActiveObject {
    friend class TMTCManager;
        
public:
    /* Constructor: sets the RF driver to use and creates the output buffer*/
    Sender(Gamma868* gamma) {
        this->gamma = gamma;
        this->outBuffer = new CircularBuffer(TMTC_OUT_BUFFER_SIZE);
    }

    /* Deconstructor */
    ~Sender() {}
    
protected:
    /* 
     * Function executed in a separate thread: reads from the outBuffer and sends
     * on the gamma868 module.
     */
    void run();

private:
    Gamma868* gamma;
    CircularBuffer* outBuffer;
};

#endif