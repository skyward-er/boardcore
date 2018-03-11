#include "SensorList.h"
#include "Writer.h"

void SlowWriteToSD(const uint8_t* data, size_t len)
{
    (void)data;
    usleep(1000 * ( len / 3 + 50 + rand() % 50));
    for(size_t i=0;i<len;i++)
        printf("%02x", data[i]);
    printf("\n");
}

int main()
{
    srand(time(NULL));
    // Invoke this before any logging
    sWriter.SetSDWriteFunction(SlowWriteToSD);
    while(1)
    {
        float x = rand() / (float)RAND_MAX;
        sWriter.LogSensor(SENSOR_TEST_1, rand(), x);
        usleep((5 + rand() % 2) * 1000);
    }
    return 0;
}
