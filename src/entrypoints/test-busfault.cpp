#include <Common.h>

using namespace miosix;

void busfault(void *useless)
{
    printf("If you can read this, Mr. BusFault is not here.\n");
}

int main()
{
    Thread::create(busfault, 1024, miosix::MAIN_PRIORITY, NULL);
    while(1)
        Thread::sleep(10);
    return 0;
}
