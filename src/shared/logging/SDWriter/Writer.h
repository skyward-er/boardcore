#ifndef SKYWARD_WRITER_H
#define SKYWARD_WRITER_H

#include <iostream>
#include <cstring>
#include <pthread.h>
#include <unistd.h>
#include "SensorList.h"

#pragma pack(1)
struct SensorData
{
    long long tick;
    float value;
    SensorId id; // you can also use just uint16_t

    SensorData(long long tick, float value, SensorId id) :
        tick(tick), value(value), id(id) {}
};

#pragma pack()

class Writer
{
    typedef void(*WriteHandler)(const uint8_t*, size_t);
public:
    static Writer& Instance();

    void SetSDWriteFunction(WriteHandler handler);
    // LogSensor is Thread-Safe!
    void LogSensor(SensorId id, long long tick, float value);
private:
    // "sizeof(SensorData) * number_of_values" OR fixed value (e.g. 1024)
    static constexpr size_t msCapacity = sizeof(SensorData) * 100;
    static constexpr size_t msSwapAtCapacity = msCapacity / 5 * 3;

    size_t mBufferOccupiedPortion, mWriteBufOccupiedPortion;

    bool mStopWritingThread;
    bool mOKToWrite;
    pthread_cond_t mWriteCondVar;
    pthread_mutex_t mSwapMutex, mSetWriteFuctionMutex;

    uint8_t *mFirstBuffer, *mWriteBuffer;
    pthread_t mWriterThreadPointer;
    WriteHandler mSDWriteHandler;

    Writer();

    ~Writer();

    void CheckIfSwappable();
    void InternalWrite(const void* data, size_t size);

    static void* ThreadLauncher(void *arg);

    void WriterThread();

    Writer(const Writer&) = delete;
    void operator=(const Writer&) = delete;
};

#define sWriter Writer::Instance()

#endif /* ifndef SKYWARD_WRITER_H */
