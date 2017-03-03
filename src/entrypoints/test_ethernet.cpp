
#include <Common.h>
#include <ethernet/UdpManager.h>
#include <string>

using namespace miosix;
using namespace std;

int main()
{
     uint8_t ipAddr[] = {192,168,1,30};                        //Device's IP address
     uint8_t mask[] = {255,255,255,0};                         //Subnet mask
     uint8_t destIp[] = {192,168,1,180};                        //Destination IP address
     
    char message[] = "message from the hell...\n";
     
    W5200& eth = W5200::instance();
    eth.setIpAddress(ipAddr);
    eth.setSubnetMask(mask);
     
    auto udp = Singleton<UdpManager>::getInstance();    
    
    udp->setTxPort(1234);
    udp->setRxPort(1235);

    int counter = 0;
    
    while(1)        
    {
        int ncar = sprintf(message,"Hello gumby n° %d!\n",counter);
        udp->sendPacketTo(destIp,2020,(void *)message,ncar);
        counter++;
        
        if(udp->newReceivedPackets()) {
            
            size_t pktSiz = udp->recvPacketSize() + 1;
            uint8_t buffer[pktSiz];
            memset(buffer,0x00,pktSiz);
            uint8_t sip[4] = {0};
            uint16_t sport = 0;
            
            udp->readPacket(sip, sport, buffer);
            printf("received %s from %d.%d.%d.%d:%d\n\n",buffer,sip[0],sip[1],sip[2],sip[3],sport);
            
        }
        
        Thread::sleep(200);
    }

    return 0;
}