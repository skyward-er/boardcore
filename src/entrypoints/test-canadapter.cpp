#include "boards/Homeone/CanAdapter/CanAdapter.h"

using namespace HomeoneBoard;
using namespace CanEventAdapter;

class TestHandler : public EventHandler
{
public:
	TestHandler() {};
	 ~TestHandler() {};

	void handleEvent(const Event& ev) override 
	{
		TRACE("[TestHandler] Received Event: %d\n", ev.sig);
	}
};



int main() 
{
	/* Create the fake handler */
	TestHandler* handler = new TestHandler();
	handler->start();

	/* Get the adapter instance */
	CanAdapter* adapter = sCanAdapter;

	while(1) 
	{
		sCanAdapter->post(Event{EV_IGN_GET_STATUS});
		miosix::Thread::sleep(500);
	}
}