
#include <Common.h>
#include <ethernet/UdpSocket.h>

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
    
    UdpSocket sock(2020);

    float arr[2] = {0,0};
    
    while(1)        
    {
        arr[0] += 10.0;
        arr[1] += 0.10;
        sock.sendTo(destIp,1234,arr,sizeof(arr));
        Thread::sleep(20);
    }
    
    return 0;
}