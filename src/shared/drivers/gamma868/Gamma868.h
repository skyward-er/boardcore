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

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#ifdef _MIOSIX
#include <miosix.h>
using namespace miosix;
#endif  //_MIOSIX

#include "CircularBuffer.h"
#include "gamma_config.h"  //Defines are in here.

class Gamma868
{
public:
    Gamma868(const char *serialPath);

    /*
     * Starts a thread that reads from the buffer and sends to the gamma module.
     */
    void start();

    /*
     * Message goes in a queue (non blocking).
     * Returns number of bytes effectively stored in the buffer.
     */
    unsigned int send(unsigned int msg_len, const char *msg);

    /*
     * Message is sent as soon as possible (blocking).
     */
    bool sendCmd(int cmd_len, const char *cmd);

    /*
     * Read from the Blocking.
     * Returns true if the received stream is a command.
     */
    bool receive(int bufLen, char *buf);

    /*
     * Set a new configuration to gamma.
     * Returns true if the configuration was set right.
     */
    bool configure(Configuration newConf);
    //~Gamma868();
    

    /*
     * TODO:
     * bool isConnected() checks if learn switch is pulled up??
     * char *readConfiguration();
     */

private:
    int fd;
    CircularBuffer outBuffer;
    Thread *writerThread;

    pthread_mutex_t readingMutex;
    pthread_mutex_t writingMutex;

    FastMutex ledMutex;
    ConditionVariable ledCond;
    int pktSent = 0;

    FastMutex learnMutex;
    ConditionVariable learnCond;
    int learnMode = 0;

    void writerThreadTask();
    void waitForLed();

    bool enterLearnMode();
    bool exitLearnMode();
    void confirmLearnMode();
    void timer();
    void printConfig();
    void waitForOk();
    int writeConfig(struct Configuration conf);

    /*
     * Static wrapper for running it in a thread.
     */
    static void *static_writerThreadTask(void *object)
    {
        reinterpret_cast<Gamma868 *>(object)->writerThreadTask();
        return 0;
    }

    /*
     * Static wrapper for running it in a thread.
     */
    static void *static_waitForLed(void *object)
    {
        reinterpret_cast<Gamma868 *>(object)->waitForLed();
        return 0;
    }

    /*
     * Static wrapper for running it in a thread.
     */
    static void *static_confirmLearnMode(void *object)
    {
        reinterpret_cast<Gamma868 *>(object)->confirmLearnMode();
        return 0;
    }

    /*
     * Static wrapper for running it in a thread.
     */
    static void *static_timer(void *object)
    {
        reinterpret_cast<Gamma868 *>(object)->timer();
        return 0;
    }
};

#endif /* GAMMA868_H */
