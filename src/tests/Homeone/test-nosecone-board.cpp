#include <boards/Nosecone/NoseconeManager.h>
#include <boards/Homeone/PinObserver.h>

using namespace NoseconeBoard;
using namespace miosix;

NoseconeManager* mgr;
PinObserver* obs;

#define finecorsaOpenBus GPIOG
#define finecorsaOpenPin 11
#define finecorsaCloseBus GPIOG
#define finecorsaClosePin 11

typedef Gpio<finecorsaOpenBus, finecorsaBus> finecorsaOpen;
typedef Gpio<finecorsaBus, finecorsaBus> finecorsa;

void fineCorsaHandler(unsigned int p, unsigned char c) {
  mgr->postEvent(Event{NoseconeBoard::EV_MOTOR_LIMIT});
}

int main()
{
  finecorsa.mode(Mode::INPUT);
  finecorsa.mode(Mode::INPUT);

  pinObs = new PinObserver();
  mgr = new NoseconeManager();

  pinObs->observePin(finecorsaBus, finecorsaPin, FALLING_EDGE, fineCorsaHandler);

  mgr->start();
  pinObs->start();

  while(1) 
  {
  	printf("What do you want the nosecone to do?\n");
    int whattodo;
    scanf("%d", &whattodo);

    if (whattodo=='o') {
      mgr->postEvent(Event{NoseconeBoard::EV_OPEN});
    }
    else if (whattodo=='c') {
      mgr->postEvent(Event{NoseconeBoard::EV_CLOSE});
    } 
    else if (whattodo=='s') {
      mgr->postEvent(Event{NoseconeBoard::EV_NC_STOP});
    }
  }

    return 0;
}
