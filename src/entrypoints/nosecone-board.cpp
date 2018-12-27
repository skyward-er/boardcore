#include <boards/Nosecone/FSM/NoseconeManager.h>
#include <events/EventBroker.h>

#include <boards/Nosecone/Events.h>
#include <boards/Nosecone/Topics.h>

#include <boards/Nosecone/LogProxy/LogProxy.h>
#include <boards/CanInterfaces.h>
#include <boards/Nosecone/Canbus/CanImpl.h>
#include <PinObserver.h>


using namespace NoseconeBoard;
using namespace miosix;
using namespace CanInterfaces;

PinObserver pinObs;
MotorDriver motor(&pinObs);

CanManager canMgr(CAN1);
NoseconeManager noseconeMgr(motor);
LoggerProxy* logger;

void noseconeInit()
{
    /* Global environment */
    sEventBroker->start();
    logger = Singleton<LoggerProxy>::getInstance();

    /* Canbus management */
    CanImpl::initCanbus(canMgr);

    /* Active Objects */
    pinObs.start();
    noseconeMgr.start();
}

int main()
{
    noseconeInit();

    TRACE("-----NOSECONE TEST---------\n");
    TRACE("  o - open\n");
    TRACE("  c - close\n");
    TRACE("  s - stop current action\n");
    TRACE("  r - read status\n");
    TRACE("  p <id> <topic> - post any event\n");
    TRACE("----------------------------\n");

    while(1) 
    {
        #ifdef DEBUG
        TRACE("What do you want the nosecone to do?\n");
        char cmd;
        scanf("%c", &cmd);

        switch(cmd) 
        {
            case 'o':
                sEventBroker->post(Event{NoseconeBoard::EV_OPEN}, TOPIC_NOSECONE);
                break;

            case 'c':
                sEventBroker->post(Event{NoseconeBoard::EV_CLOSE}, TOPIC_NOSECONE);
                break;

            case 's':
                sEventBroker->post(Event{NoseconeBoard::EV_STOP}, TOPIC_NOSECONE);
                break;

            case 'p': 
            {
                uint8_t id, topic;
                TRACE("enter <id> <topic>: ");

                scanf("%c %c", &id, &topic);
                sEventBroker->post(Event{id}, topic);

                TRACE("\n");
                break;
            }

            case 'r': 
            {
                NoseconeBoardStatus status = logger->getNoseconeStatus();

                TRACE("Status:\n");
                for(unsigned int i = 0; i < sizeof(NoseconeBoardStatus); i++)
                {
                    TRACE("%d ", reinterpret_cast<uint8_t*>(&status)[i] );
                }
                TRACE("\n");
            }

            default:
                TRACE("Unknown option\n");
                break;
        }

        #else
        miosix::Thread::sleep(1000);
        #endif
    }


    return 0;
}
