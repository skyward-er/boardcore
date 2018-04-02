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
 * Run() function of the Receiver:
 * parse the received packet one byte at a time until you find a complete mavlink
 * message, then forward the corresponding event.
 */
void Receiver::run() {
	mavlink_message_t msg;
	mavlink_status_t status;
	uint8_t byte;

	while(1)
	{
		gamma->receive(&byte, 1); //Blocking function

		if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
		{
			printf("Received message with ID %d, sequence: %d from component %d of system %d",
								msg.msgid, msg.seq, msg.compid, msg.sysid);

			switch(msg.msgid) {
			    case MAVLINK_MSG_ID_PING:
			    {
			        mavlink_ping_t ping;
			        mavlink_msg_ping_decode(&msg ,&ping);
			        printf("Received ping\n");
			        break;
			    }
			    case MAVLINK_MSG_ID_TEST_MSG:
			    {
			        mavlink_test_msg_t ack;
			        mavlink_msg_test_msg_decode(&msg, &ack);
			        printf("Received ack\n");
			        break;
			    }
			}

		}
	}
}


/* 
 * Run() function of the Sender: 
 * read from the out buffer and write on the RF serial. 
 */
void Sender::run() {
   uint8_t msgTemp[MAX_PKT_SIZE];
   uint32_t readBytes;
   bool sent;

	while(1) {
	    if (outBuffer->occupiedSize() > 0) {
	    	/* Read from the buffer at maximum MAX_PKT_SIZE bytes */
	        readBytes = outBuffer->read(msgTemp, MAX_PKT_SIZE);

	        /* Try sending the packet for a limited number of times */
	        for (int i = 0; i < MAX_TRIES_PER_PACKET; i++) {
	        	sent = gamma->send(msgTemp, readBytes);
	        	if(sent) break;
	    	}
	    }
	    Thread::sleep(TMTC_SEND_TIMEOUT);
	}

}


/* TMTCManager Constructor: initialise objects (has memory allocation) */
TMTCManager::TMTCManager() {
    gamma = new Gamma868("/dev/tty");
    //TODO: check gamma status and configuration

    sender = new Sender(gamma);
    receiver = new Receiver(gamma);
}

/* TMTCManager non-blocking send() function */
bool TMTCManager::send(uint8_t* msg, uint8_t len) {
	/* Check if there's enough free space in the Sender's outBuffer */
    if(sender->outBuffer->freeSize() >= len){
        sender->outBuffer->write(msg, len);
        return true;
    } else {
        return false;
    }   
}