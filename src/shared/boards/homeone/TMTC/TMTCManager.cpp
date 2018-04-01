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
	uint8_t byte;

	while(1)
	{
		gamma->receive(1, &byte); //Blocking function

		if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg))
		{
			printf("Received message with ID %d, sequence: %d from component %d of system %d",
								 msg.msgid, msg.seq, msg.compid, msg.sysid);
		}
	}
}


/* Run() function of the Sender: read from the out buffer and write on the driver */
void Sender::run(){
   uint8_t msg[100];

	while(1){
	    while (outBuffer->size() > 0) 
	    {
	        outBuffer->read(msg, outBuffer->size());
	        printf ("%d\n", msg[0]);
	    }
	}
}


/* TMTCManager Constructor: has some memory allocation */
TMTCManager::TMTCManager(){
    gamma = new Gamma868("/dev/tty");
    //TODO: check gamma status and configuration

    sender = new Sender(gamma);
    receiver = new Receiver(gamma);
}