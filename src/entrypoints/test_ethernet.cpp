
#include <Common.h>
#include <ethernet/UdpManager.h>

using namespace miosix;
using namespace std;

int main()
{    
    udpComm->start();

    while(1)
    {
        ledOn();
        Thread::sleep(1000);
        ledOff();
        Thread::sleep(1000);
    }
    
    
    return 0;
}