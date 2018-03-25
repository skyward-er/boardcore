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
#include <Gamma868.h>
#include "SyncQueue.h"

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

class TMTCReceiver : ActiveObject {
    
public:
    TMTCReceiver(Gamma868 gamma){
	this.gamma = gamma;
    }
    
    ~TMTCReceiver() {}
    
protected:
    run() override{
	message_t message;
	gamma.receive(sizeof(message_t), &message);
	msg_type type = message.type;
	
	printf("RECEIVED MESSAGE %d\n", type);
    }
    

private:
    Gamma868 gamma;
    
};

class TMRCSender : ActiveObject {
        
public:
    TMRCSender(Gamma868 gamma){
	this.gamma = gamma;
    }
    
    addToBuffer(message_t* message){
	outBuffer.put(message);
	
    }
    
    ~TMRCSender() {}
    
protected:
    run() override{
	
	message_t toSend = NULL;
	toSend = outBuffer.get();
	
	if(toSend != NULL){
	    gamma.send();
	}
    }
    
private:
    SynchronizedQueue<message_t> outBuffer;
    Gamma868 gamma;
    
};


class TMTCManager : public Singleton<TMTCManager>
{

    friend class Singleton<TMTCManager>;

public:
    ~TMTCManager() {}
    
    void send(message_t msg){
	sender.addToBuffer(&msg);
    }
    
    void setActive(bool status){
	this.status = status;
    }

protected:


private:
    TMTCManager(){
	printf("TMTC Built\n");
    }
    
    bool status;
    Gamma868 gamma("/dev/auxtty");
    
    TMRCSender sender(gamma);
    TMTCReceiver receiver(gamma);
   
};

#define sTMTCManager TMTCManager::getInstance()

#endif /* ifndef TMTCMANAGER_H */