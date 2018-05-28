#include <Common.h>
#include "boards/Homeone/TMTCManager/TMTCManager.h"
#include "boards/Homeone/StatusManager/StatusManager.h"

using namespace miosix;
using namespace HomeoneBoard;
using namespace TMTC;
using namespace Status;

int main()
{

	sStatusManager;
	printf("Created Status Manager");

	return 0;
}
