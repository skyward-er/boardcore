#include <miosix.h>
#include <utils/Debug.h>
#include <diagnostic/PrintLogger.h>

using namespace miosix;
using namespace Boardcore;

int main()
{
    PrintLogger logger = Logging::getLogger("main");

    LOG_INFO(logger, "This is an info message");
    LOG_ERR(logger, "This is an error message");

    return 0;
}
