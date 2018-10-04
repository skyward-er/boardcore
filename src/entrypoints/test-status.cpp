#include <Common.h>
#include "boards/Homeone/TMTCManager/TMTCManager.h"
#include "boards/Homeone/StatusManager/StatusManager.h"

using namespace miosix;
using namespace HomeoneBoard;
using namespace TMTC;
using namespace Status;

int main()
{
	printf("Creating Status Manager\n");
	sStatusManager;
	printf("Created Status Manager\n");

	while(1) {
		Event ev = { EV_DEBUG_INFO_REQUEST };
		sEventBroker->post(ev, TOPIC_DIAGNOSTICS);
		Thread::sleep(1000);
	}

	return 0;
}
