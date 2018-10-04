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

#include "TMTC_Config.h"
#include <Singleton.h>
#include "CircularBuffer.h"
#include <drivers/gamma868/Gamma868.h>
#include "boards/Homeone/Events.h"
#include "boards/Homeone/Topics.h"
#include "events/EventBroker.h"  
#include "boards/Homeone/StatusManager/Status.h"

namespace HomeoneBoard
{
namespace TMTC
{

/*
 * Status of the component, wraps the mavlink stats
 * which are updated at every byte reception.
 */
struct TMTCStatus : ComponentStatus
{
    mavlink_status_t mavstatus;
    uint8_t sendErrors;
    uint8_t ackErrors;

	TMTCStatus()
	{
		compId = TMTC_COMP_ID;
		healthStatus = COMP_UNINIT;
	}
};

/**
 * The TMTCManager class handles the communication with the Ground Station.
 * It uses a Gamma868 transceiver and implements the Mavlink protocol.
 *
 * SENDER:
 * Since the transceiver's driver defines blocking functions and the LoRa
 * protocol used by the Gamma can be very slow, a synchronized CircularBuffer
 * is used to store outgoing messages: the Sender thread will then periodically
 * send them over the link.
 *
 * RECEIVER:
 * A Receiver thread is also used to read the Gamma868 buffer byte-per-byte and
 * then, when an entire message is read, an appropriate handler defined in
 * MessageHandlers.h is called.
 *
 * USE:
 * At the end of the file you can find the sTMTCManager define.
 */
class TMTCManager : public Singleton<TMTCManager>
{
    friend class Singleton<TMTCManager>;

public:
    /**
     * Class destructor.
     */
    ~TMTCManager() {}

    /**
     * \return a copy of the status struct.
     */
    TMTCStatus getStatus()
    {
        return status;
    }

    /**
     * Non-blocking function that can be used to send a message: copies the
     * message into a synchronized buffer.
     * Note that the message should already be a Mavlink message.
     * \param  msg       buffer that contains the message
     * \param  len       length of the message in bytes
     * \return           false if there isn't enough space in the buffer
     */
    bool enqueueMsg(const uint8_t* msg, const uint8_t len);

protected:
private:

    TMTCStatus status;
    Gamma868* gamma;
    CircularBuffer* outBuffer;

    miosix::Thread* senderThread;
    miosix::Thread* receiverThread;

    /**
     * Private constructor that realizes the Singleton pattern.
     */
    TMTCManager();

    /**
     * Calls the runSender() member function.
     * \param arg       this object's pointer, casted to void*
     */
    static void senderLauncher(void* arg)
    {
        reinterpret_cast<TMTCManager*>(arg)->runSender();
    }

    /**
     * Calls the runReceiver() member function.
     * \param arg       this object's pointer, casted to void*
     */
    static void receiverLauncher(void* arg)
    {
        reinterpret_cast<TMTCManager*>(arg)->runReceiver();
    }

    /**
     * Function ran by the sending thread:
     * looks for messages in the outBuffer an sends them through the link
     * using the module's driver.
     */
    void runSender();

    /**
     * Function ran by the receiving thread:
     * read and parse incoming messages from the module's buffer and handle them
     * according to their content.
     */
    void runReceiver();


    /**
     *  Handle the Mavlink message, posting the corresponding event if needed.
     * \param msg           pointer to the mavlink message to handle
     */
    void handleMavlinkMessage(const mavlink_message_t* msg);

    /**
     * Send an acknowledge message back to the sender to notify the Ground
     * Station that you correctly received a message.
     * \param msg    Mavlink message to acknowledge.
     */
    void sendAck(const mavlink_message_t* msg);
};

} /* namespace HomeoneBoard */
} /* namespace TMTC */

/* Define a singleton object that can be accessed from other files */
#ifndef sTMTCManager
#define sTMTCManager HomeoneBoard::TMTC::TMTCManager::getInstance()
#else
#error TMTCMANAGER ALREADY DEFINED
#endif /* sTMTCManager */

#endif /* TMTCMANAGER_H */
