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

struct Configuration{
    int local_addr[3] = {127, 127, 127};
    int dest_addr[3] = {127, 127, 127};
    int lora_mode = 6;
    int lora_pow = 15;
    int handshake = 0;
    int baudrate = 4;
};

class Gamma868 {
    public:
        Gamma868(const char *serialPath);
        bool send(const char *msg);
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
        //pthread_t thread;
        pthread_mutex_t readingMutex;
        pthread_cond_t readingCond;
        pthread_mutex_t writingMutex;
        pthread_cond_t writingCond;
};

#endif /* GAMMA868_H */