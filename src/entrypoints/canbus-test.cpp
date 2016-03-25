#include <Common.h>
#include <canbus/CanManager.h>
#include <canbus/CanSocket.h>
#include <canbus/CanUtils.h>
#include <sensors/MPU9250.h>

using namespace std;
using namespace miosix;

#define CAN_PACKETID 0x49
#ifdef SENDBOARD
    #define NAME "Send Board"
#else
    #define NAME "Recv Board"
#endif

int main() {
    CanManager c(CAN1);

    canbus_init_t st = {
        CAN1, Mode::ALTERNATE,  9, {CAN1_RX0_IRQn,CAN1_RX1_IRQn}
    };
    c.addBus<GPIOA_BASE, 11, 12>(st);
    //canbus_init_t st2= {
    //    CAN2, Mode::ALTERNATE,  9, {CAN2_RX0_IRQn,CAN2_RX1_IRQn}
    //};
    //c.addBus<GPIOB_BASE, 5, 6>(st2);

    CanBus* bus = c.getBus(0);

    cout << NAME << endl;
    cout << "*** Ready ***" << endl;


#ifdef SENDBOARD
    while(1) {
        const char *pkt = "TestMSG";
        cout << "Sending pkt" << endl;
        bus->send(CAN_PACKETID, (const uint8_t *)pkt, strlen(pkt));
        Thread::sleep(1000);
    }
#else
    char buf[64]={0};
    CanSocket socket(CAN_PACKETID);
    socket.open(bus);

    while(1) {
        socket.receive(&buf, 64);
        cout << "Recv pkt: '" << buf << "'" << endl;
    }

    socket.close();
#endif
}
