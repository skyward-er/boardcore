
#include <Common.h>
#include <ethernet/UdpManager.h>

using namespace miosix;
using namespace std;

int main()
{
     uint8_t ipAddr[] = {192,168,1,30};                        //Device's IP address
     uint8_t mask[] = {255,255,255,0};                         //Subnet mask
     uint8_t destIp[] = {192,168,1,4};                        //Destination IP address
     
    unsigned char message[] = "message from the hell...\n";
     
    W5200& eth = W5200::instance();
    eth.setIpAddress(ipAddr);
    eth.setSubnetMask(mask);
     
    auto udp = Singleton<UdpManager>::getInstance();    
    
    udp->setPort(1234);

    while(1)        
    {
        udp->sendPacketTo(destIp,2020,(void *)message,sizeof(message)/sizeof(unsigned char));
        Thread::sleep(1000);
    }

    return 0;
}