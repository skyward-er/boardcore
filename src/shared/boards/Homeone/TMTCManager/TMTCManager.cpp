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

#include "TMTCManager.h"

/* 
 * Constructor: initialise objects (has memory allocation).
 */
TMTCManager::TMTCManager() {
    gamma = new Gamma868(RADIO_DEVICE_NAME);
    outBuffer = new CircularBuffer(TMTC_OUT_BUFFER_SIZE);
    //TODO: check gamma status and configuration
    senderThread = miosix::Thread::create(senderLauncher, TMTC_SENDER_STACKSIZE, TMTC_SENDER_PRIORITY,
                                        reinterpret_cast<void*>(this));
    receiverThread = miosix::Thread::create(receiverLauncher, TMTC_RECEIVER_STACKSIZE, TMTC_RECEIVER_PRIORITY,
                                        reinterpret_cast<void*>(this));
}

/* 
 * Non-blocking send function: copies the message in the outBuffer if there's enough space.
 */
bool TMTCManager::enqueueMsg(uint8_t* msg, uint8_t len) {
    if(outBuffer->freeSize() >= len){
        outBuffer->write(msg, len);
        return true;
    }

    return false; 
}

/* 
 * Sending thread's run() function: read from the outBuffer and forward on the link.
 */
void TMTCManager::runSender() {
   uint8_t msgTemp[TMTC_MAX_PKT_SIZE];

	while(1) {
	    if (outBuffer->occupiedSize() > 0) {
	    	// Read from the buffer at maximum MAX_PKT_SIZE bytes
	        uint32_t readBytes = outBuffer->read(msgTemp, TMTC_MAX_PKT_SIZE);

	        // Try sending the packet (multiple times)
	        for (int i = 0; i < TMTC_MAX_TRIES_PER_PACKET; i++) {
	        	bool sent = gamma->send(msgTemp, readBytes);
	        	if(sent)
	        		break;
	    	}
	    }
	    miosix::Thread::sleep(TMTC_SEND_TIMEOUT);
	}

}

/* 
 * Receiving thread's run() function: parse the received packet one byte at a time 
 * until you find a complete mavlink message and halde it with the appropriate handler.
 */
void TMTCManager::runReceiver() {
	mavlink_message_t msg;
	mavlink_status_t status;
	uint8_t byte;

	while(1)
	{
		gamma->receive(&byte, 1); //Blocking function

		// Parse one char at a time until you find a complete message 
		if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
		{
			printf("Received message with ID %d, sequence: %d from component %d of system %d",
								msg.msgid, msg.seq, msg.compid, msg.sysid);

			// If the received message is not an ACK, send an ACK back to ground 	
			if(msg.msgid != MAVLINK_MSG_ID_ACK)
				sendAck(&msg);

			// Handle the message depending on the message type 
			switch(msg.msgid) {
			    case MAVLINK_MSG_ID_PING:
			    {
			        mavlink_ping_t ping;
			        mavlink_msg_ping_decode(&msg ,&ping);
			        TCHandler::handlePing(&ping);
    			    break;
			    }
			}

		}
	}
}

/* 
 * Send an ACK to notify the sender that you received the given message.
 */
void TMTCManager::sendAck(mavlink_message_t* msg){
	// Create ack message 
	mavlink_message_t ack_msg;
	mavlink_msg_ack_pack(1, 1, &ack_msg, msg->msgid, msg->seq);

	// Send message back to the sender through the callback 
	bool ackSent = enqueueMsg( (uint8_t*)&ack_msg, sizeof(ack_msg) );
	if(!ackSent) {
        //TODO: fault counter? retry?
    } 
}