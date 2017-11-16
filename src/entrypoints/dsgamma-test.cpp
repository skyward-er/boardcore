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

/* DISCOVERY F429I*/
typedef Gpio<GPIOG_BASE,13> greenLed;
typedef Gpio<GPIOG_BASE,14> redLed;
typedef Gpio<GPIOA_BASE,0> button;


Gamma868 gamma("/dev/auxtty"); //create object

long long sendTime = 0;
int nTentativi = 0;
int tot = 0;
int state = 0; //0 = normal, 1 = sender, 2 = echo receiver

void btnClick(void *arg);
void stdInput(void *arg);
void handleCommand(char *cmd);
void handleData(char *data);
void receiver();

int main() {
    gamma.start(); //!!!!!IMPORTANT!!!!!!!!
    
    //Discovery gpio setup
    {
        FastInterruptDisableLock dLock;
        greenLed::mode(Mode::OUTPUT);
        redLed::mode(Mode::OUTPUT);
        button::mode(Mode::INPUT);
    }
    
    //STACK_DEFAULT_FOR_PTHREAD is needed for printf())
    Thread::create(btnClick, STACK_DEFAULT_FOR_PTHREAD);
    Thread::create(stdInput, STACK_DEFAULT_FOR_PTHREAD);
    receiver();     //Runs in a loop
    
    return 0;
}

void btnClick(void *arg){
    
    char msg[CMD_LEN];
    
    while(1){
        
        //Wait for button
        while(1){
            if(button::value()==1) break;       
        }
        
        //---------------Send COMMAND----------------
        printf("Writing CMD... \n");
        msg[0] = (char) 10;
        gamma.sendCmd(CMD_LEN, msg);
        //------------------------------------------
        state = 1;
        
        printf("Ok \n" );
        Thread::sleep(200);
    }
}

void stdInput(void *arg){
    
    char msg[DATA_LEN + 1];
    
    while(1){
        
        //Wait for button
        scanf("%s", msg);
        
        //Save current time
        if(state == 1){
            sendTime = miosix::getTick();
            nTentativi++;
        }
        
        //----------------SEND DATA----------------
        gamma.send(strlen(msg), msg);
        //-----------------------------------------
        
        printf("Ok \n" );
        Thread::sleep(200);
    }
}

/*
 * Continuosusly reads from the device.
 */
void receiver(){
    
    char inputBuf[DATA_LEN];
        
    while(1){ 
        
        //Read PKT_LEN bytes
        printf("Reading: \n");
        bool cmdReceived = gamma.receive(DATA_LEN, inputBuf);
        
        if(cmdReceived) 
            handleCommand(inputBuf);
        else
            handleData(inputBuf);
    }
}

void handleCommand(char *cmd){
    printf("Command received!\n");
    switch((int)cmd[0]){
                case 0: 
                    state = 0;
                    break;
                case 10:
                    state = 2;
                    break;
                default:
                    printf("Unknown command\n");
            }
}

void handleData(char *data){

    long long arrivalTime = miosix::getTick();
    printf("Received: %s\n", data);
    

    if(state == 2){
        gamma.send(DATA_LEN, data);
    }  else if (state == 1) {
        int rtt = arrivalTime - sendTime;
        printf("RTT: %d\n", rtt);

        printf("--------------RESULT------------");
        tot += rtt;

        //Print Delay calculation
        if(nTentativi > 0)
        printf("Tentativi: %d  Media: %d \n", nTentativi, tot/(2*nTentativi));

    }
}