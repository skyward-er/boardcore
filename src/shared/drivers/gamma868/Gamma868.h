/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Nuno Barcellos
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

#include <Common.h>
#include <fcntl.h>

#ifdef _MIOSIX
#include <miosix.h>
using namespace miosix;
#endif  //_MIOSIX

#include "gamma_config.h"  //Defines are in here.

class Gamma868
{
public:
    /*
     * Create a Gamma868 object using the given path as the serial port to use.
     * @param serialPath        Name of the serial port (es. /dev/tty)
     */
    Gamma868(const char* serialPath);

    /*
     * Send a message through the serial port to the gamma868 module (blocking).
     * @param pkt               Pointer to the packet (needs to be at least pkt_len bytes).
     * @param pkt_len           Lenght of the packet to be sent.
     * @return                  True if the message was sent correctly.
     */
    bool send(const uint8_t* pkt, uint32_t pkt_len);

    /*
     * Send a message through the serial port to the gamma868 module (blocking).
     * @param pkt               Pointer to the buffer (needs to be at least pkt_len bytes).
     * @param pkt_len           Lenght of the packet to be received.
     */
    void receive(uint8_t* pkt, uint32_t pkt_len);

    /*
     * Set a new configuration to the gamma868 module.
     * @retun                   True if the configuration was set correctly.
     */
    bool config(Configuration newConf);    

    /*
     * TODO:
     * bool isConnected() checks if learn switch is pulled up??
     * Configuration readConfiguration();
     */

private:
    int fd;

    FastMutex gammaMutex;

    FastMutex ledMutex;
    ConditionVariable ledCond;
    int pktSent = 0;

    FastMutex learnMutex;
    ConditionVariable learnCond;
    int learnMode = 0;

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
    static void* static_confirmLearnMode(void *object)
    {
        reinterpret_cast<Gamma868 *>(object)->confirmLearnMode();
        return 0;
    }

    /*
     * Static wrapper for running it in a thread.
     */
    static void* static_timer(void *object)
    {
        reinterpret_cast<Gamma868 *>(object)->timer();
        return 0;
    }
};

#endif /* GAMMA868_H */