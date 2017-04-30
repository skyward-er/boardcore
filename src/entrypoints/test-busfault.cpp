#include <Common.h>

using namespace miosix;

void busfault(void *useless)
{
    printf("Hello, i'm a bus fault!\n");
}

int main()
{
    Thread::create(busfault, 1024, miosix::MAIN_PRIORITY, NULL);
    while(1)
        Thread::sleep(10);
    return 0;
}
