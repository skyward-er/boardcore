
#include <Common.h>
#include <BusTemplate.h>
#include "drivers/stm32f2_f4_i2c.h"
#include <events/Scheduler.h>


using namespace miosix;

void supercar(void *arg);


template<typename P>
void blinker()
{
    if(P::value()) P::low(); else P::high();
}

int main() {
    //Thread *ledTh=Thread::create(supercar,STACK_MIN);

    sEventScheduler->add(blinker<leds::led9>,100);
    sEventScheduler->add(blinker<leds::led8>,200);
    sEventScheduler->add(blinker<leds::led7>,500);
    sEventScheduler->add(blinker<leds::led6>,1000);
    sEventScheduler->add(blinker<leds::led5>,50);
    for(;;){
        Thread::sleep(10);
    }


}





void supercar(void *arg)
{
    vector<GpioPin> pins={
        leds::led9::getPin(),
        leds::led8::getPin(),
        leds::led7::getPin(),
        leds::led6::getPin(),
        leds::led5::getPin(),
        leds::led4::getPin(),
        leds::led3::getPin(),
        leds::led2::getPin()
    };
    size_t cur=0;
    size_t prev=0;
    enum {
        FORWARD,
        BACKWARD
    } dir=FORWARD;
    for(;;)
    {
        switch(dir)
        {
            case FORWARD:
                if(cur==pins.size()-1) dir=BACKWARD;
                else cur++;
                break;
            case BACKWARD:
                if(cur==0) dir=FORWARD;
                else cur--;
                break;
        }
        pins.at(prev).low();
        Thread::sleep(100);
        pins.at(cur).high();
        prev=cur;
        Thread::sleep(100);
    }
}
