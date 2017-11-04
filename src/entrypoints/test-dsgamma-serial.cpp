
#include <cstdio>
#include "miosix.h"
//#include <pthread.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


using namespace std;
using namespace miosix;

typedef Gpio<GPIOD_BASE,12> greenLed;
typedef Gpio<GPIOD_BASE,14> redLed;

void thread1(void *arg);

int main(){
     
    {
        FastInterruptDisableLock dLock;
        greenLed::mode(Mode::OUTPUT);
        redLed::mode(Mode::OUTPUT);
    }
    
    int fd = open("/dev/auxtty",O_RDWR);
    if(fd<0) printf("Error opening serial test port\n");
    
    Thread *t;
    t=Thread::create(thread1,STACK_MIN);
    if(t==NULL) printf("Error: thread not created\n");
    
    for(;;){
        greenLed::high();
        Thread::sleep(200);
        greenLed::low();
        Thread::sleep(2000);
        printf("Hey!\n");
        
        write(fd,"24",2);
    }
}

void thread1(void *arg){
    int i = 0;
    for(;;){
        redLed::high();
        Thread::sleep(50);
        redLed::low();
        Thread::sleep(50);
        i++;
        if(i==4){
            Thread::sleep(2000);
            i = 0;
        }
    }
}