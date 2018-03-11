#include "Writer.h"

Writer& Writer::Instance()
{
    static Writer instance;
    return instance;
}

/*
 * (Synchronized on mSetWriteFuctionMutex)
 * Sets the function to be executed when the first buffer is full.
 * Should be set before logging.
 * 
 * \arg function to be executed, of the type WriteHandler defined in the header.
 */
void Writer::SetSDWriteFunction(WriteHandler handler)
{
    pthread_mutex_lock(&mSetWriteFuctionMutex);
    mSDWriteHandler = handler;
    pthread_mutex_unlock(&mSetWriteFuctionMutex);
}

/*
 * Wraps the sensor's data in a struct and logs it using the InternalWrite() function.
 * 
 * \arg id id of the sensor, of type SensorId which is an enum define externally.
 * \arg tick time of logging.
 * \arg value sensor's value that need to be logged.
 */
void Writer::LogSensor(SensorId id, long long tick, float value)
{
    SensorData data(tick, value, id);
    InternalWrite(&data, sizeof(data)); // Thread-Safe
}

/*
 * Constructor:  initializes all variables and mutex and creates the
 * SDWrite thread.
 */
Writer::Writer() : mBufferOccupiedPortion(0), mWriteBufOccupiedPortion(0),
        mStopWritingThread(false), mOKToWrite(false),
        mSDWriteHandler(NULL)
{
    pthread_mutex_init(&mSwapMutex, NULL);
    pthread_mutex_init(&mSetWriteFuctionMutex, NULL);
    pthread_cond_init(&mWriteCondVar, NULL);

    mFirstBuffer = new uint8_t[msCapacity];
    mWriteBuffer = new uint8_t[msCapacity];

    pthread_create(&mWriterThreadPointer, NULL, &Writer::ThreadLauncher, this);
    std::cout << "Max capacity: " << msCapacity << std::endl;
}

/*
 * Deconstructor: joins the writing thread and destroys all variables and mutex.
 */
Writer::~Writer()
{
    mStopWritingThread = true;
    pthread_cond_signal(&mWriteCondVar);
    pthread_join(mWriterThreadPointer, NULL);

    delete[] mFirstBuffer;
    delete[] mWriteBuffer;

    pthread_mutex_destroy(&mSwapMutex);
    pthread_mutex_destroy(&mSetWriteFuctionMutex);
    pthread_cond_destroy(&mWriteCondVar);
}

/*
 * Check if the buffer is occupied enough to be wrote on the SD card:
 * if it is, signal the WriteCondVar condition variable.
 * Called by InternalWrite().
 */
void Writer::CheckIfSwappable()
{
    static_assert(msSwapAtCapacity < msCapacity, "Are you stupid?");
    if(mBufferOccupiedPortion >= msSwapAtCapacity)
    {
        mOKToWrite = true;
        pthread_cond_signal(&mWriteCondVar);
    }
}

/*
 * (Synchronized on SwapMutex)
 * Logs a sensor's data if there's enough space in the buffer.
 * 
 * \arg data pointer to the data.
 * \arg size size of data.
 */
void Writer::InternalWrite(const void* data, size_t size)
{
    pthread_mutex_lock(&mSwapMutex);
    CheckIfSwappable();

    if(mBufferOccupiedPortion + size < msCapacity)
    {
        memcpy(&mWriteBuffer[mBufferOccupiedPortion], data, size);
        mBufferOccupiedPortion += size;
    }
    else
        std::cout << "DROPPPPPPPP\n"; // drop packet

    pthread_mutex_unlock(&mSwapMutex);
}

/*
 * Static wrapper for the WriterThread function.
 * \arg Writer object that will run the WriterThread() function.
 */
void* Writer::ThreadLauncher(void *arg)
{
    ((Writer *)arg)->WriterThread();
    pthread_exit(NULL);
    return 0;
}

/*
 * (Synchronized on SwapMutex and SetWriteFuctionMutex)
 * Waits for the WriteCondVar condition variable to be signaled,
 * then swaps the the buffers and writes the first buffer on the SD card.
 */
void Writer::WriterThread()
{
    std::cout << "Thread started\n";
    while(!mStopWritingThread)
    {
        pthread_mutex_lock(&mSwapMutex);
        pthread_cond_wait(&mWriteCondVar, &mSwapMutex);

        if(!mOKToWrite || mStopWritingThread)
        {
            pthread_mutex_unlock(&mSwapMutex);
            continue;
        }

        // Write to SD
        std::swap(mFirstBuffer, mWriteBuffer);
        std::swap(mBufferOccupiedPortion, mWriteBufOccupiedPortion);
        mOKToWrite = false;

        pthread_mutex_unlock(&mSwapMutex);

        if(mWriteBufOccupiedPortion > 0)
        {
            pthread_mutex_lock(&mSetWriteFuctionMutex);
            if(mSDWriteHandler != NULL)
                mSDWriteHandler(mWriteBuffer, mWriteBufOccupiedPortion);
            pthread_mutex_unlock(&mSetWriteFuctionMutex);
            mWriteBufOccupiedPortion = 0;
        }
    }
    std::cout << "Thread stopped\n";
}
