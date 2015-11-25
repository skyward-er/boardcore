#include <Common.h>
#include <canbus/CanManager.h>
#include <canbus/CanSocket.h>
#include <canbus/CanUtils.h>

using namespace std;
//using namespace mxgui;
using namespace miosix;

typedef Gpio<GPIOA_BASE,11> canRX;
typedef Gpio<GPIOA_BASE,12> canTX;

#define CAN_SENDID 0x49
#define CAN_RECVID 0x49
#ifdef SENDBOARD
    #define NAME "Send Board"
#else
    #define NAME "Recv Board"
#endif
    
list<string> blog;
FastMutex renderMutex;
int n_upd = 0;
int my_upd = 0;
void render() {
    Lock<FastMutex> l(renderMutex);

    if(my_upd == n_upd)
        return;
    my_upd = n_upd;

    while(blog.size() > 0) {
        const string &s = *blog.begin();
        cout << s << endl;
        blog.pop_front();
    }

/*
    int y = 10;
    DrawingContext dc(Display::instance());
    dc.clear(0);
    for(string s : blog) {
        dc.write(Point(10,y), s.c_str()); 
        y += 13;
    }
*/
}

void log(const string& str) {
    Lock<FastMutex> l(renderMutex);
    blog.push_back(str);
    //while(blog.size() >= 24)
    //    blog.pop_front();
    ++n_upd;
}

void render(void *arg) {
    while(1) {
        render();
        Thread::sleep(50);
    }
}
void tt(void *arg) {
    int msg_size;
    char message[32];

    CanSocket *socket = static_cast<CanSocket *>(arg);
    while(1) {
        log("Waiting for message...");
        memset(message,0,sizeof(message));
        socket->receive(message, msg_size);
        message[31] = 0;
        log(string("Message: *") + message + "*");
    }
}

int main() {
    log(NAME);
    log("*** Ready ***");
    //InputHandler& backend = InputHandler::instance();

    canRX::mode(Mode::ALTERNATE);
    canRX::alternateFunction(9);

    canTX::mode(Mode::ALTERNATE);
    canTX::alternateFunction(9);

    CanManager *manager = CanManager::getCanManager(CAN1);

    CanSocket socket;

#ifdef SENDBOARD
    socket.open(manager, CAN_RECVID);
#else
    socket.open(manager, CAN_SENDID);
#endif

    Thread *thread, *rthread;
    thread = Thread::create(tt,2048,1,&socket,Thread::JOINABLE);
    rthread= Thread::create(render,2048,1,NULL,Thread::DEFAULT);
    while(1) {

        #ifdef SENDBOARD
            /*while(1) {
                Event e = backend.popEvent();
                if(e.getEvent() == EventType::ButtonA)
                    break;
                miosix::Thread::sleep(20);
            }*/
            Thread::sleep(3000);

            log("Sending message...");
            socket.send("abcdefgh", 8, CAN_SENDID);
        #else
            thread->join();
        #endif
    }

    socket.close();

    return 0;
}
