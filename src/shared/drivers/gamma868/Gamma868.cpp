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

#include "Gamma868.h"

using namespace std;

/*
 * A serial port attached to the Gamma868 RX and TX pins is expected 
 * to be passed to the object in order to communicate with the device.
 * NOTE that the serial port has to be already open and at the baudrate
 * at which the module has been configured (default is 9600 baud).
 * 
 * The object also uses 2 other pins, gammaSwitch and gammaLed, which are
 * defined in the gamma_config.h file. 
 */
Gamma868::Gamma868(const char *serialPath)
{
    fd=open(serialPath,O_RDWR);
    if(fd<0) printf("Cannot open %s\n", serialPath);
    gammaLed::mode(Mode::INPUT);
    
}


/*
 * Adds data to circular buffer (non blocking).
 * Returns how many cahrs could be effectively stored in the buffer.
 */
int Gamma868::send(int msg_len, const char *msg)
{ 
    int retVal = 0;
    if(last == -1) return false;
    else
    {
        int i = 0;
        pthread_mutex_lock(&bufMutex);
        for(i = 0; i < msg_len; i++){
            buffer[last] = msg[i];
            last++;
            if(last == MAX_BUFFER) last = 0;
            if(last == first){
                last =-1;
                break;
            }
        }
        pthread_mutex_unlock(&bufMutex);
        
        retVal = i;
    }
    return retVal;
}

/*
 * Immediately sends command (blocking).
 */
bool Gamma868::sendCmd(int cmd_len, const char *cmd){
    
    //TODO implement command length
    char pkt[HEAD_LEN + cmd_len + END_LEN];

    //Prepare packet
    pkt[0] = START;
    pkt[1] = CMD;
    pkt[2] = cmd[0];
    pkt[HEAD_LEN + CMD_LEN] = END;

    //Send to gamma (synchronized)
    pthread_mutex_lock(&writingMutex); 
        write(fd, pkt, HEAD_LEN + cmd_len + END_LEN);
    pthread_mutex_unlock(&writingMutex);  

    return true;
}

/*
 * Reads from the gamma868 serial.
 */
bool Gamma868::receive(int bufLen, char *buf)
{
    char init = (char) 0;
    char type = (char) 0;
    char end = (char) 0;
    bool cmd = false;
    
    //Read what you received (synchronized ?)
    //First byte: start
    while(init != START){
        read(fd, &init, 1);
    }
    
    pthread_mutex_lock(&readingMutex);
        //Second byte: type of data
        read(fd, &type, 1);
        if(type == DATA){
            read(fd, buf, bufLen);
        }else if(type == CMD){
            read(fd, buf, CMD_LEN);
            cmd = true;
        }
        //End byte
        read(fd, &end, 1);        
    pthread_mutex_unlock(&readingMutex);
    
    if(end!= END) 
        printf("Did not find end char! stream may be inconsistent\n");

    return cmd;
}

/*
 * Reads from the circular buffer and, if there's something in the buffer,
 *  sends it in packets of fixed dimension, then until the end of transmission.
 */
void Gamma868::readFromBuffer(){
    while(1){                                   //Continuously check the buffer
        
    if(bufSize() > MIN_TO_SEND){                         //If there's something in it...
       
        while(bufSize() > 0){                   //Read and send all the buffer
            //Prepare header
            char pkt[HEAD_LEN + DATA_LEN + END_LEN];
            pkt[0] = START;
            pkt[1] = DATA;
            
            pthread_mutex_lock(&bufMutex);      //Lock the buffer
                int nChar = bufSize() > 64 ? 64 : bufSize();
                
                //Copy from circular buffer
                for(int i = 0; i < nChar; i++){
                    pkt[i+HEAD_LEN] = buffer[first];
                    first++;
                    if(first == MAX_BUFFER)
                        first = 0;
                    if(first == last){
                        break;
                    }
                }    
            pthread_mutex_unlock(&bufMutex);

            pkt[DATA_LEN + HEAD_LEN] = END;

            //Start thread that will notify the end of the message sending
            Thread::create(&Gamma868::static_waitForLed, 
                        STACK_DEFAULT_FOR_PTHREAD,
                        MAIN_PRIORITY,
                        reinterpret_cast<void*>(this));
            //Send
            pthread_mutex_lock(&writingMutex); 

                write(fd, pkt, HEAD_LEN + DATA_LEN + END_LEN);

                {
                    Lock<FastMutex> l(ledMutex);
                    while(sent==0) ledCond.wait(l);
                    sent = 0;
                }
            pthread_mutex_unlock(&writingMutex);
        }
        Thread::sleep(200); //TODO needed?
    }
    
    }  
}

/*
 * Static wrapper for running it in a thread.
 */


void Gamma868::waitForLed(){
    bool sending = false;
    while(1){
        if(gammaLed::value() == 1 && !sending){
            sending = true;
        }
        if(sending && gammaLed::value() == 0){
            Lock<FastMutex> l(ledMutex);
            sent++;
            ledCond.signal();
            break;
        }
    }
}

/*
 * Calculates the occupied portion of the circular buffer.
 */
unsigned int Gamma868::bufSize(){
    if(last == -1)
        return MAX_BUFFER;
    else if(last < first)
        return MAX_BUFFER - (first - last);
    else
        return last - first;
}