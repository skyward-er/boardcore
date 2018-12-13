#include <boards/Nosecone/NoseconeManager.h>
#include <events/EventBroker.h>

using NoseconeBoard::FMM::NoseconeManager;
using namespace miosix;

NoseconeManager* mgr;

int main()
{
  mgr = sNoseconeManager;
  mgr->start();
  sEventBroker->start();

  int whattodo;
  int o=1;
  int c=0;

	printf("What do you want the nosecone to do?\n");
  scanf("%d", &whattodo);


  if (whattodo==o) {
    mgr->postEvent(Event{NoseconeBoard::EV_OPEN});
    miosix::ledOn();
  }
  else if (whattodo==c) {
    mgr->postEvent(Event{NoseconeBoard::EV_CLOSE});
  }

	while(1) {
		sleep(2000);
	}

    return 0;
}
