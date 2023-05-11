#include <diagnostic/PrintLogger.h>
#include <miosix.h>
#include <utils/Debug.h>

#include <fstream>
#include <sstream>

using namespace miosix;
using namespace Boardcore;

int main()
{
    std::ofstream file("/sd/testfile.txt", std::ios::app);

    if (file.is_open())
    {
        file << "Hi mom!" << std::endl;

        file.close();

        while (true)
        {
            ledOn();
            Thread::sleep(500);
            ledOff();
            Thread::sleep(500);
        }
    }
    else
    {
        while (true)
        {
            ledOn();
            Thread::sleep(100);
            ledOff();
            Thread::sleep(100);
        }
    }

    return 0;
}
