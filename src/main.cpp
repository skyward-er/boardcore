#include <Common.h>
#include <canbus/CanManager.h>
#include <canbus/CanSocket.h>
#include <canbus/CanUtils.h>

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

    canbus_init_t st = {CAN1, Mode::ALTERNATE,  9};
    c.addBus<GPIOA_BASE, 11, 12>(st);
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
    char buf[64];
    CanSocket socket(CAN_PACKETID);
    socket.open(bus);

    while(1) {
        socket.receive(&buf, 64);
        cout << "Recv pkt: '" << buf << "'" << endl;
    }

    socket.close();
#endif
}
