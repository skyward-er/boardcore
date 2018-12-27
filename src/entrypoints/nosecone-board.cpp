#include <boards/Nosecone/NoseconeBoard.h>


using namespace NoseconeBoard;
using namespace miosix;
using namespace CanInterfaces;

void printHelp()
{
    TRACE("-----NOSECONE BOARD---------\n");
    TRACE("  o - open\n");
    TRACE("  c - close\n");
    TRACE("  s - stop current action\n");
    TRACE("  r - read status\n");
    TRACE("  p <id> - post any event\n");
    TRACE("  h - print this menu\n");
    TRACE("-----------------------------\n");
}

NoseconeBoardImpl board;

int main()
{
    //board = new NoseconeBoardImpl();

    printHelp();

    while(1) 
    {
        #ifdef DEBUG
        TRACE("Command: ");
        char cmd;
        scanf("%c", &cmd);
        printf("\n");

        switch(cmd) 
        {
            case 'o':
                board.postEvent(Event{NoseconeBoard::EV_OPEN});
                break;

            case 'c':
                board.postEvent(Event{NoseconeBoard::EV_CLOSE});
                break;

            case 's':
                board.postEvent(Event{NoseconeBoard::EV_STOP});
                break;

            case 'p': 
            {
                uint8_t id;
                TRACE("enter <id>: ");

                scanf("%c", &id);
                board.postEvent(Event{id});

                TRACE("\n");
                break;
            }

            case 'r': 
            {
                NoseconeBoardStatus status = board.getStatus();

                TRACE("Status:\n");
                for(unsigned int i = 0; i < sizeof(NoseconeBoardStatus); i++)
                {
                    TRACE("%d ", reinterpret_cast<uint8_t*>(&status)[i] );
                }
                TRACE("\n");
            }

            case 'h':
                printHelp();
                break;

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
