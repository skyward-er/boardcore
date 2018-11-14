#include <Common.h>
#include "boards/Homeone/Events.h"

#include "boards/Homeone/IgnitionController/IgnitionController.h"

#include <iostream>
using namespace miosix;
using namespace HomeoneBoard;
using namespace IGN;
int main()
{
    sIgnitionController->start();
    sEventBroker->start();

    //CanAbstraction canAbst(LINK_IGNITION);
    //ignitionPublisher = canAbst.getPublisher(CAN_IGNITION_STATUS);
    //ignitionStatusSub = canAbst.getSubscriber(CAN_IGNITION);


	while(1) {
		Thread::sleep(100);
        
        StartLaunchEvent evt;
        evt.launchCode = 1;
        sEventBroker->post(evt, TOPIC_IGNITION);

     //   ignitionPublisher->publish((uint8_t*) &launchCode, sizeof(can_abort_ignition_msg));
	}

	return 0;
}
