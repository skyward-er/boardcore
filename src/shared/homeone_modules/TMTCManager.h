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

#ifndef TMTCMANAGER_H
#define TMTCMANAGER_H

#include <ActiveObject.h>
#include <Common.h>
#include <Singleton.h>
#include <drivers/gamma868/Gamma868.h>

class TMTCReceiver;
class TMTCSender;

#define PAYLOAD_LEN 10

enum msg_type 
{
    TM,
    STATUS_TC,
    STATUS_RESPONSE,
    PING_TC,
    PING_RESPONSE
};

typedef struct {
    msg_type type;
    uint8_t data[PAYLOAD_LEN];
} message_t;


class TMTCManager : public Singleton<TMTCManager>
{

    friend class Singleton<TMTCManager>;

public:
    ~TMTCManager() {}
    
    void send(message_t* msg);
    
    //void setActive(bool st);

protected:


private:
    TMTCManager();
    
    bool status;
    Gamma868* gamma;
    
    TMTCSender* sender;
    TMTCReceiver* receiver;
   
};

#define sTMTCManager TMTCManager::getInstance()

#endif /* ifndef TMTCMANAGER_H */