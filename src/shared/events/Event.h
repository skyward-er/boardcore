/*
 * Event.h
 *
 *  Created on: 11 mar 2017
 *      Author: matteo
 */

#ifndef EVENT_H_
#define EVENT_H_

enum Signal{
	ENTRY,
	EXIT,
    FIRST_SIGNAL
};


struct Event{
	uint8_t sig;
};

/* Example of extended Event structure
struct ExtendedEvent : public Event{
	uint32_t custom_event;
};
*/



#endif /* EVENT_H_ */
