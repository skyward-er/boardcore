#include <boards/Homeone/Events.h>
#include "boards/Homeone/CanInterfaces.h"
#include "drivers/canbus/can_events/CanEventAdapter.h"

using namespace HomeoneBoard;
using namespace CanInterfaces;
#define DEBUG

/**
 * Master EventHandler
 */
class TestHandler : public EventHandler
{

public:
    /* Subscribe to new can msgs on the ignition topic */
    TestHandler(CanEventAdapter& can_ev_adapter, uint16_t topic)
        : can_ev_adapter(can_ev_adapter)
    {
        canSocket = can_ev_adapter.subscribe(this, topic, EV_NEW_CAN_MSG);
        printf("[TestHandler] Created Can Socket\n");
    }
    ~TestHandler(){};

    /* Process an event */
    void handleEvent(const Event& ev) override
    {
        printf("[TestHandler] Received Event: %d\n", ev.sig);

        /* If it's a message notification */
        if (ev.sig == EV_NEW_CAN_MSG)
        {
            /* Read all the message queue */
            while (canSocket->haveMessage())
            {
                char msg[CAN_MAX_LEN + 1] = {'\0'};

                canSocket->receive(msg, CAN_MAX_LEN);

                printf("[TestHandler] CanMsg: %s\n", msg);
            }
        }
    }

private:
    CanEventSocket* canSocket;
    CanEventAdapter& can_ev_adapter;
};

/**s
 * Master board main
 */
int main()
{
    CanEventAdapter* can_ev_adapter = new CanEventAdapter();

    TestHandler* rcv = new TestHandler(*can_ev_adapter, CAN_TOPIC_IGNITION);
    rcv->start();

    rcv->postEvent(Event{10});

    // uint8_t msg[7] = {'S','K','Y','W','A','R','D'};
    // sCanEventAdapter->postMsg(msg, 7, TOPIC);
    uint8_t buf[CAN_MAX_PAYLOAD];
    while (1)
    {
        printf("Sending... ");
        /* Send event on the canbus */
        int l   = canMsgSimple(buf, 0x0A);
        bool ok = can_ev_adapter->postMsg(buf, l, CAN_TOPIC_HOMEONE);
        printf("ok=%d\n", ok);

        miosix::Thread::sleep(500);
    }
}
