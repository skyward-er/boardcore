#include "drivers/pwm/pwm.h"
#include "Common.h"


using namespace miosix;

int main()
{
	Pwm* pwm = new Pwm(1, 0.9);

	pwm->configure(1, 1);
	pwm->start();

    while(1)
    {
        printf("End\n");
        Thread::sleep(1000);
    }

    return 0;
}
