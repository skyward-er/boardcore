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

#define PKT_LEN 24

typedef Gpio<GPIOG_BASE,13> greenLed;
typedef Gpio<GPIOG_BASE,14> redLed;
typedef Gpio<GPIOA_BASE,0> button;

int ping = 0;
Gamma868 gamma("/dev/auxtty"); 

long long sendTime = 0;
int nTentativi = 0;
int tot = 0;

void sender(void *arg);
void receiver();
int main() {
    
    //Discovery gpio setup
    {
        FastInterruptDisableLock dLock;
        greenLed::mode(Mode::OUTPUT);
        redLed::mode(Mode::OUTPUT);
        button::mode(Mode::INPUT);
    }
    
    Thread::create(sender, STACK_MIN);
    
    receiver();
    
    
    return 0;
}

void sender(void *arg){
    while(1){
        
        while(1){
            if(button::value()==1) break;       //Wait for button
        }
        
        printf("Writing... \n");
        
        char msg[PKT_LEN];
        for(int i = 0; i < PKT_LEN; i++){
            msg[i] = '#';
        }
        
        sendTime = miosix::getTick();
        nTentativi++;
        gamma.send(msg);
        
        printf("Ok \n" );
        Thread::sleep(200);
    }
}

void receiver(){
    while(1){
        //read 
        // int len = 1; // SENDER ONLY
        int len = PKT_LEN;
        char inputBuf[len];
        printf("Reading: \n");
        gamma.receive(len, inputBuf);
        
        for(int i = 0; i < len ; i++){
            printf("Received: %c\n", inputBuf[i]);
        }
        
        long long arrivalTime = miosix::getTick();
        int rtt = arrivalTime - sendTime;
        printf("RTT: %d\n", rtt);
        
        printf("--------------RESULT------------");
        tot += rtt;
        printf("Tentativi: %d  Media: %d \n", nTentativi, tot/(2*nTentativi));
        
        // RECEIVER ONLY
        Thread::sleep(200);
        gamma.send(inputBuf);
         
        
        //Thread::sleep(1000);
    }
}