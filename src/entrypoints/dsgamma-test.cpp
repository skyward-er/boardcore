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

#include "miosix.h"
#include <drivers/gamma868/Gamma868.h>

using namespace std;
using namespace miosix;


/*
 * STRANGE THING #1 (AKA BUG):
 * Stack overflow if PKT_LEN in greater than 24 !!!!!
 */
#define PKT_LEN 14

typedef Gpio<GPIOG_BASE,13> greenLed;
typedef Gpio<GPIOG_BASE,14> redLed;
typedef Gpio<GPIOA_BASE,0> button;

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
    
    receiver();     //Runs in a loop
    
    
    return 0;
}

void sender(void *arg){
    
    char msg[PKT_LEN];
    
    while(1){
        
        //Wait for button
        while(1){
            if(button::value()==1) break;       
        }
        
        //Prepare a packet of PKT_LEN chars with all #
        printf("Writing... \n");
        for(int i = 0; i < PKT_LEN; i++){
            msg[i] = '#';
        }
        
        //Save current time
        sendTime = miosix::getTick();
        nTentativi++;
        
        //Send
        gamma.send(msg);
        
        printf("Ok \n" );
        Thread::sleep(200);
    }
}

void receiver(){
    
    int len = PKT_LEN;
    char inputBuf[len];
    long long arrivalTime;
        
    while(1){ 
        
        //Read PKT_LEN bytes
        printf("Reading: \n");
        gamma.receive(len, inputBuf);
        
        //Print received chars
        for(int i = 0; i < len ; i++){
            printf("Received: %c\n", inputBuf[i]);
        }
        
        //Calculate Round Trip Time
        arrivalTime = miosix::getTick();
        int rtt = arrivalTime - sendTime;
        printf("RTT: %d\n", rtt);
        
        printf("--------------RESULT------------");
        tot += rtt;
        
        //Print Delay calculation
        if(nTentativi > 0)
        printf("Tentativi: %d  Media: %d \n", nTentativi, tot/(2*nTentativi));
        
        //------------------ RECEIVER ONLY -----------------
        gamma.send(inputBuf);
        //--------------------------------------------------
    }
}