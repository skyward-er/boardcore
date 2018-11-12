#include "drivers/canbus/can_events/CanEventAdapter.h"
#include <boards/Homeone/Events.h>
#include "boards/Homeone/CanInterfaces.h"

using namespace HomeoneBoard;
using namespace CanInterfaces;

#define TOPIC_RCV canTopicToInt(CanTopic::CAN_TOPIC_COMMANDS)
#define TOPIC_SND canTopicToInt(CanTopic::CAN_TOPIC_IGN)
#define DEBUG

/**
 * Master EventHandler
 */
class TestHandler : public EventHandler
{

public:
	/* Subscribe to new can msgs on the ignition topic */
	TestHandler(uint16_t topic) 
	{
		canSocket = sCanEventAdapter->subscribe(this, topic, EV_NEW_CAN_MSG);
		printf("[TestHandler] Created Can Socket\n");
	}
	~TestHandler() {};

	/* Process an event */
	void handleEvent(const Event& ev) override 
	{
		printf("[TestHandler] Received Event: %d\n", ev.sig);

		/* If it's a message notification */
		if(ev.sig == EV_NEW_CAN_MSG) 
		{
			/* Read all the message queue */
			while(canSocket->haveMessage()) 
			{
				char msg[CAN_MAX_LEN + 1] = {'\0'};

				canSocket->receive(msg, CAN_MAX_LEN);

				printf("[TestHandler] CanMsg: %s\n", msg);
			}
		}
	}

private:
	CanEventSocket* canSocket;
};


/**
 * Master board main
 */
int main() 
{
	TestHandler* rcv = new TestHandler(TOPIC_SND);
	rcv->start();

	rcv->postEvent(Event{10});

	// uint8_t msg[7] = {'S','K','Y','W','A','R','D'};
	// sCanEventAdapter->postMsg(msg, 7, TOPIC);

	while(1) 
	{
		printf("Sending... ");
		/* Send event on the canbus */
		bool ok = sCanEventAdapter->postEvent(Event{0xAA}, TOPIC_RCV);
		printf("ok=%d\n", ok);

		miosix::Thread::sleep(500);
	}
}

