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

#ifndef GAMMA868_H
#define GAMMA868_H

#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#ifdef _MIOSIX

#include <miosix.h>

using namespace miosix;

#endif //_MIOSIX   

#include "gamma_config.h"

class Gamma868 {
    public:
        Gamma868(const char *serialPath);
        
        /*
         * Starts a thread that reads from the buffer and sends to the gamma module.
         */
        void start(){
            Thread::create(&Gamma868::static_readFromBuffer,
                              STACK_DEFAULT_FOR_PTHREAD,
                              MAIN_PRIORITY,
                              reinterpret_cast<void*>(this));
        }
        
        /*
         * Message goes in a queue (non blocking).
         * Returns number of bytes effectively stored in the buffer.
         */
        int send(int msg_len, const char *msg);
        
        /*
         * Message is sent as soon as possible (blocking).
         */
        bool sendCmd(int cmd_len, const char *cmd);
        
        /*
         * Read from the Blocking.
         * Returns true if the received stream is a command.
         */
        bool receive(int bufLen, char *buf);
        //~Gamma868();
        
        /*
         * TODO:
         * bool isConnected() checks if learn switch is pulled up??
         * bool enterLearnMode();
         * char *readConfiguration();
         * bool configure(Configuration newConf);
         * void exitLearnMode();
         */
        
    private:
        int fd;
        pthread_t writeThread;
        pthread_mutex_t readingMutex;
        pthread_mutex_t writingMutex;
        pthread_mutex_t bufMutex;
        
        FastMutex ledMutex;          
        ConditionVariable ledCond; 
        int sent=0;
        
        int first = 0;
        int last = 0;
        char buffer[MAX_BUFFER];
        
        unsigned int bufSize();
        
        void readFromBuffer();
        void waitForLed();
        
                
        static void* static_readFromBuffer(void * object){
            reinterpret_cast<Gamma868*>(object)->readFromBuffer();
        }
        
        static void* static_waitForLed(void * object){
             reinterpret_cast<Gamma868*>(object)->waitForLed();
        }
        
        void printBufContent(){
            printf("Buffer: first %d last %d occupied %u total size %d\n", 
                    first, last, bufSize(), MAX_BUFFER);
        
            int offset = first;
            for (int i = 0; i < MAX_BUFFER; i++){
                if (offset + i == MAX_BUFFER) offset = -i;
                printf("%c", buffer[i]);
            }
            printf("\n");
        }
};

#endif /* GAMMA868_H */