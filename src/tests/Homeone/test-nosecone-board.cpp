#include <boards/Nosecone/NoseconeManager.h>
#include <PinObserver.h>

using namespace NoseconeBoard;
using namespace miosix;

NoseconeManager* mgr;
PinObserver* pinObs;

typedef Gpio<GPIOG_BASE, 11> r_finecorsa;
typedef Gpio<GPIOG_BASE, 11> l_finecorsa;

void r_finecorsaHandler(unsigned int p, unsigned char c) {
  mgr->postEvent(Event{NoseconeBoard::EV_MOTOR_LIMIT});
}

void l_finecorsaHandler(unsigned int p, unsigned char c) {
  mgr->postEvent(Event{NoseconeBoard::EV_MOTOR_LIMIT});
}


int main()
{
  r_finecorsa::mode(Mode::INPUT);
  l_finecorsa::mode(Mode::INPUT);

  mgr = new NoseconeManager();
  mgr->start();

  pinObs = new PinObserver();
  pinObs->observePin(GPIOG_BASE, 11, PinObserver::Trigger::FALLING_EDGE, r_finecorsaHandler);
  pinObs->observePin(GPIOG_BASE, 12, PinObserver::Trigger::FALLING_EDGE, l_finecorsaHandler);
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
