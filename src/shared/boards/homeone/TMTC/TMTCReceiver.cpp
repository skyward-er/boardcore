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

#include "TMTCReceiver.h"

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

		/* Parse one char at a time until you find a complete message */
		if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
		{
			printf("Received message with ID %d, sequence: %d from component %d of system %d",
								msg.msgid, msg.seq, msg.compid, msg.sysid);

			/* If the received message is not an ACK, send an ACK back to ground */			
//			TODO: create ACK message
//			if(msg.msgid != MAVLINK_MSG_ID_ACK){
//				sendAck(&msg);
//			}

			/* Handle the message depending on the message type */
			switch(msg.msgid) {
			    case MAVLINK_MSG_ID_PING:
			    {
			        mavlink_ping_t ping;
			        mavlink_msg_ping_decode(&msg ,&ping);
			        handlePing(&ping);
			    }
			    break;
			    case MAVLINK_MSG_ID_TEST_MSG:
			    {
			        mavlink_test_msg_t test;
			        mavlink_msg_test_msg_decode(&msg, &test);
			        handleTestMsg(&test);
			    }
			    break;
			}

		}
	}
}

/* TODO
 * Send an ACK to notify the sender that you received the given message.
 
void Receiver::sendAck(mavlink_message_t* msg){
	// Create ack message 
	mavlink_message_t ack_msg;
	mavlink_msg_ack_pack(SYS_ID,COMPONENT_ID, &ack_msg, msg->msgid, msg->seq);

	// Send message back to the sender through the callback 
	//TODO: callback.send(&ack_msg, sizeof(mavlink_message_t));	
}

*/

/* PING message handler */
void Receiver::handlePing(mavlink_ping_t* msg) {
	printf("Received ping\n");
}

/* TEST_MSG message handler */
void Receiver::handleTestMsg(mavlink_test_msg_t* msg) {
	printf("Received test message\n");
	
}