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

#ifndef TMTCMANAGER_H
#define TMTCMANAGER_H

#include <ActiveObject.h>
#include <Common.h>
#include <Singleton.h>
#include <drivers/gamma868/Gamma868.h>
#include "CircularBuffer.h"
#include <libs/mavlink_skyward_lib/mavlink_lib/skyward/mavlink.h>

#ifndef TMTC_OUT_BUFFER_SIZE
#define TMTC_OUT_BUFFER_SIZE 256
#endif

/*
 * The Receiver class is an ActiveObject that reads incoming packets
 * from the RF module and forwards an event to the EventBroker accordingly to
 * the received message.
 */
class Receiver : ActiveObject {
    friend class TMTCManager;
    
public:
    /* Constructor: sets the RF driver to use */
    Receiver(Gamma868* gamma) {
        this->gamma = gamma;
    }
    
    /* Deconstructor */
    ~Receiver() {}
    
protected:
    /* Function executed in a separate thread */
    void run();

private:
    Gamma868* gamma;

};

/*
 * The Sender class is an ActiveObject that forwards on the RF module 
 * the packets that are in its queue using the driver's blocking functions.
 */
class Sender : ActiveObject {
    friend class TMTCManager;
        
public:
    /* Constructor: sets the RF driver to use */
    Sender(Gamma868* gamma) {
        this->gamma = gamma;
        this->outBuffer = new CircularBuffer(TMTC_OUT_BUFFER_SIZE);
    }

    /* Deconstructor */
    ~Sender() {}
    
protected:
    /* Function executed in a separate thread */
    void run();

private:
    Gamma868* gamma;
    CircularBuffer* outBuffer;
};


/*
 * The TMTCManager class handles the communication with the Ground Station
 * through the Gamma868 RF module driver.
 *
 * It is composed of:
 *  - the Gamma driver, which uses only blocking functions.
 *  - the Sender, an ActiveObject that sends packets through the driver.
 *  - the Receiver, an ActiveObject that processes the incoming packets.
 *
 * All the functions of the Manager are non-blocking.
 */
class TMTCManager : public Singleton<TMTCManager>
{

    friend class Singleton<TMTCManager>;

public:
    /*
     * Class deconstructor.
     */
    ~TMTCManager() {}
    
    /*
     * Non-blocking function to send a message through the RF module:
     * adds the message to the queue of the TMTCSender.
     * @param  msg       buffer that contains the message
     * @param  len       length of the message in bytes
     */
    void send(uint8_t* msg, uint8_t len){
        sender->outBuffer->write(msg, len);
    }

protected:

private:
    /*
     * Private constructor to implement the Singleton pattern.
     */
    TMTCManager();
    
    /* Gamma RF driver */
    Gamma868* gamma;
    
    /* Sender ActiveObject */
    Sender* sender;
    /* Receiver ActiveObject */
    Receiver* receiver;
   
};

/* Define to access the singleton object from other files */
#define sTMTCManager TMTCManager::getInstance()

#endif /* ifndef TMTCMANAGER_H */