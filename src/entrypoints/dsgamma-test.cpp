/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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

#include <cstdio>
#include "miosix.h"
#include <drivers/gamma868/Gamma868.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;
using namespace miosix;

#define MSG_LEN 6

typedef Gpio<GPIOG_BASE,13> greenLed;
typedef Gpio<GPIOG_BASE,14> redLed;
typedef Gpio<GPIOA_BASE,0> button;

int ping = 0;
Gamma868 gamma("/dev/auxtty"); 

long long sendTime = 0;
int nTentativi = 0;
int tot = 0;

void sender();
void receiver(void *arg);
int main() {
    
    //Discovery gpio setup
    {
        FastInterruptDisableLock dLock;
        greenLed::mode(Mode::OUTPUT);
        redLed::mode(Mode::OUTPUT);
        button::mode(Mode::INPUT);
    }
    
    sender();
    
    
    return 0;
}

void sender(){

    while(1){
        
        while(1){
            if(button::value()==1) break;       //Wait for button
        }
        
        printf("Writing... \n");
        
        nTentativi++;
        if(nTentativi == 1) Thread::create(receiver, STACK_MIN);

        char msg[MSG_LEN];
        for(int i = 0; i < nTentativi; i++){
            msg[i] = '#';
        }
        
        sendTime = miosix::getTick();
        gamma.send(msg);
        
        printf("Ok \n" );
        Thread::sleep(200);
    }
}

void receiver(void *arg){
    
    while(1){
        //read 
        char inputBuf[MSG_LEN];
        printf("Reading: \n");
        gamma.receive(MSG_LEN, inputBuf);
        
        for(int i = 0; i < MSG_LEN ; i++){
            printf("Received: %c\n", inputBuf[i]);
        }
        
        long long arrivalTime = miosix::getTick();
        int rtt = arrivalTime - sendTime;
        printf("RTT: %d\n", rtt);
        
        printf("--------------RESULT------------");
        tot += rtt;
        
        if(nTentativi > 0)
        printf("Tentativi: %d  Media: %d \n", nTentativi, tot/(2 * nTentativi));
        
        // RECEIVER ONLY
        //Thread::sleep(50);
        //gamma.send(inputBuf);
         
        
        //Thread::sleep(200);
    }
}