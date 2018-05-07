/* Copyright (c) 2018 Skyward Experimental Rocketry
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

#include <Common.h>
#include <Singleton.h>
#include <drivers/gamma868/Gamma868.h>
#include <libs/mavlink_skyward_lib/mavlink_lib/skyward/mavlink.h>
#include "CircularBuffer.h"
#include "TMTC_Config.h"

/*
 * The TMTCManager class handles the communication with the Ground Station.
 * It is composed of:
 *  - the Gamma868 driver, which uses blocking functions to receive from and send
 *    to the RF module.
 *  - a Sender thread that sends packets through the driver.
 *  - a Receiver thread that processes the incoming packets.
 */
class TMTCManager : public Singleton<TMTCManager>
{
    friend class Singleton<TMTCManager>;

public:
    /*
     * Class deconstructor.
     */
    ~TMTCManager() {}
    
    /*
     * Non-blocking function that can be used to send a message: copies the
     * message into a synchronized buffer.
     * @param  msg       buffer that contains the message
     * @param  len       length of the message in bytes
     * @return           false if there isn't enough space in the buffer
     */
    bool enqueueMsg(const uint8_t* msg, const uint8_t len);

protected:

private:
    /* RF module driver */
    Gamma868* gamma;
    /* Synchronized buffer for outgoing messages */
    CircularBuffer* outBuffer;
    
    /* Pointers to sending and receiving threads */
    miosix::Thread* senderThread;
    miosix::Thread* receiverThread;
    
    /*
     * Private constructor that realizes the Singleton pattern.
     */
    TMTCManager();

    /*
     * Calls the runSender() member function
     * @param arg       the object pointer cast to void*
     */
    static void senderLauncher(void* arg)
    {
        reinterpret_cast<TMTCManager*>(arg)->runSender();
    }

    /*
     * Calls the runReceiver() member function
     * @param arg       the object pointer cast to void*
     */
    static void receiverLauncher(void* arg)
    {
        reinterpret_cast<TMTCManager*>(arg)->runReceiver();
    }

    /*  
     * Function ran by the sending thread:
     * look for messages in the outBuffer an send them through the link using
     * the module's driver.
     */
    void runSender();

    /*  
     * Function ran by the receiving thread:
     * read and parse incoming messages from the module's buffer and handle them 
     * according to their content.
     */
    void runReceiver();

    /* 
     * Send an acknowlege message back to the sender to notify the Ground Station
     * that you correctly received the message with a given sequence number.
     */
    void sendAck(const mavlink_message_t* msg);

    /* -------------------------- MESSAGE HANDLERS ------------------------- */

    /*
     * Handle Ping command.
     */
    void handlePingCommand(const mavlink_message_t* command);

    /*
     * Handle a no argument command according to command in it.
     */
    void handleNoArgCommand(const mavlink_message_t* command);

    /*
     * Handle the Launch Command.
     */
    void handleLaunchCommand(const mavlink_message_t* command);

    /*
     * Handle a Status request.
     */
    void handleStatusRequestCommand(const mavlink_message_t* command);

    /*
     * Handle the calibration command.
     */
    void handleCalibrationCommand(const mavlink_message_t* command);

    /*
     * Handle a raw_event message.
     */
    void handleRawEventMessage(const mavlink_message_t* rawEventMsg);
   
};

/* Define a singleton object that can be accessed from other files */
#ifndef sTMTCManager
#define sTMTCManager TMTCManager::getInstance()
#else
#error TMTCMANAGER ALREADY DEFINED
#endif/* ifndef sTMTCManager */

#endif /* ifndef TMTCMANAGER_H */