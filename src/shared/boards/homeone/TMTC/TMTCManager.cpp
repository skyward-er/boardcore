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

/* Run() function of the Receiver: handle incoming commands */
void Receiver::run(){
	mavlink_message_t msg;

	while(1)
	{
		uint8_t rcvByte;
		gamma.receive(1, &rcvByte); //Blocking function

		if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg))
		{
			printf("Received message with ID %d, sequence: %d from component %d of 
			                  system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
		}
	}
}


/* Run() function of the Sender: read from the out buffer and write on the driver */
void Sender::run(){
   mavlink_message_t* msg;

	while(1){
	    while (outBuffer.len() > 0) 
	    {
	        msg = outBuffer.get();
	        printf ("%d\n", msg->data[0]);
	    }
	}
}

/* Sender function to add a message to the queue */
void Sender::addToBuffer(mavlink_message_t* message){
   outBuffer.put(message);
}


/* TMTCManager Constructor: has some memory allocation */
TMTCManager::TMTCManager(){
    gamma = new Gamma868("/dev/tty");
    sender = new Sender(gamma);
    receiver = new Receiver(gamma);
}

/* TMTCManager non-blocking send() function */
void TMTCManager::send(mavlink_message_t* msg) {
	sender->addToBuffer(msg);
}
