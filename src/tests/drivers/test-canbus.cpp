#include <Common.h>
#include <drivers/canbus/CanManager.h>
#include <drivers/canbus/CanUtils.h>

using namespace std;
using namespace miosix;

#define CAN_PACKETID 0x49

void handleCan (CanMsg message, CanStatus status) 
{
    unsigned char buf[65] = {0};
    memcpy(buf, message.Data, message.DLC);
    printf("Received %s\n", buf);

    UNUSED(status);
}

int main()
{
    CanManager c(CAN1);

    canbus_init_t st = {
        CAN1, Mode::ALTERNATE, 9, {CAN1_RX0_IRQn, CAN1_RX1_IRQn}};
        
    c.addBus<GPIOA_BASE, 11, 12>(st, handleCan);
    // canbus_init_t st2= {
    //    CAN2, Mode::ALTERNATE,  9, {CAN2_RX0_IRQn,CAN2_RX1_IRQn}
    //};
    // c.addBus<GPIOB_BASE, 5, 6>(st2);

    CanBus *bus = c.getBus(0);
    c.addHWFilter(CAN_PACKETID, 0);

    printf("*** Ready ***\n");

    while (1)
    {
        ledOn();
        const char *pkt = "TestMSG";
        bus->send(CAN_PACKETID, (const uint8_t *)pkt, strlen(pkt));
        //socket.receive(buf, 64);
        Thread::sleep(250);
        ledOff();
        Thread::sleep(150);
    }

}