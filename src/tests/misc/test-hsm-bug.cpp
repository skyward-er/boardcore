#include <miosix.h>
#include "events/EventBroker.h"
#include "events/HSM.h"

using namespace miosix;

static const uint8_t TOPIC = 4;

enum Events : uint8_t
{
    EV_A = EV_FIRST_SIGNAL,
    EV_B,
    EV_C
};

/*
        S
       /\
      /  \
     S1  S2
     |   |
    S12 S21
*/
class HSMTest : public HSM<HSMTest>
{
public:
    HSMTest() : HSM(&HSMTest::state_init)
    {
        sEventBroker->subscribe(this, TOPIC);
    }

    State state_init(const Event& ev)
    {
        return transition(&HSMTest::state_S12);
    }

    State state_S(const Event& ev)
    {
        State retState = HANDLED;
        switch (ev.sig)
        {
            case EV_ENTRY:
                printf("[S] ENTRY\n");
                break;
            case EV_INIT:
                printf("[S] INIT\n");
                retState = transition(&HSMTest::state_S1);
                break;
            case EV_EXIT:
                printf("[S] EXIT\n");
                break;
            default:
                retState = tran_super(&HSMTest::Hsm_top);
        }
        return retState;
    }
    State state_S1(const Event& ev)
    {
        State retState = HANDLED;
        switch (ev.sig)
        {
            case EV_ENTRY:
                printf("[S1] ENTRY\n");
                break;
            case EV_INIT:
                printf("[S1] INIT\n");
                retState = transition(&HSMTest::state_S12);
                break;
            case EV_EXIT:
                printf("[S1] EXIT\n");
                break;
            case EV_A:
                printf("[S1] EV_A\n");
                retState = transition(&HSMTest::state_S21);
                break;
            default:
                retState = tran_super(&HSMTest::state_S);
        }
        return retState;
    }

    State state_S12(const Event& ev)
    {
        State retState = HANDLED;
        switch (ev.sig)
        {
            case EV_ENTRY:
                printf("[S12] ENTRY\n");
                break;
            case EV_EXIT:
                printf("[S12] EXIT\n");
                break;
            default:
                retState = tran_super(&HSMTest::state_S1);
        }
        return retState;
    }

    State state_S2(const Event& ev)
    {
        State retState = HANDLED;
        switch (ev.sig)
        {
            case EV_ENTRY:
                printf("[S2] ENTRY\n");
                break;
            case EV_INIT:
                printf("[S2] INIT\n");
                retState = transition(&HSMTest::state_S21);
                break;
            case EV_EXIT:
                printf("[S2] EXIT\n");
                break;
            default:
                retState = tran_super(&HSMTest::state_S);
        }
        return retState;
    }
    State state_S21(const Event& ev)
    {
        State retState = HANDLED;
        switch (ev.sig)
        {
            case EV_ENTRY:
                printf("[S21] ENTRY\n");
                break;
            case EV_EXIT:
                printf("[S21] EXIT\n");
                break;
            case EV_B:
                printf("[S21] EV_B\n");
                break;
            default:
                retState = tran_super(&HSMTest::state_S2);
        }
        return retState;
    }
};

void slp(unsigned int ms = 500) { Thread::sleep(ms); }

int main()
{
    sEventBroker->start();

    HSMTest hsm;
    hsm.start();

    slp();

    printf("Posting event\n");
    sEventBroker->post({EV_A}, TOPIC);

    slp();
    sEventBroker->post({EV_B}, TOPIC);

    while (true)
    {
        slp(10000);
    }
}