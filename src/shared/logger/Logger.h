
#pragma once

#include <cstdio>
#include <list>
#include <miosix.h>
#include "LogBase.h"

/**
 * Possible outcomes of Logger::log()
 */
enum class LogResult
{
    Queued,  ///< Data has been accepted by the logger and will be written
    Dropped, ///< Buffers are currently full, data will not be written. Sorry :(
    Ignored, ///< Logger is currently stopped, data will not be written
    TooLarge ///< Data is too large to be logged. Increase maxRecordSize
};

/**
 * Buffered logger. Needs to be started before it can be used.
 */
class Logger
{
public:
    /**
     * Blocking call. May take a long time.
     * 
     * Call this function to start the logger.
     * When this function returns, the logger is started, and subsequent calls
     * to log will actually log the data.
     */
    void start();
    
    /**
     * Blocking call. May take a very long time (seconds).
     * 
     * Call this function to stop the logger.
     * When this function returns, all log buffers have been flushed to disk,
     * and it is safe to power down the board without losing log data or
     * corrupting the filesystem.
     */
    void stop();
    
    /**
     * Nonblocking call.
     * 
     * \return true if the logger is started and ready to accept data.
     */
    bool isStarted() const { return started; }
    
    /**
     * Nonblocking call. Call this function to log a class.
     * \param lb The class to be logged, read the LogBase documentation for
     * requirements.
     * \return whether the class has been logged
     */
    LogResult log(const LogBase& lb);
    
private:
    Logger();
    Logger(const Logger&)=delete;
    Logger& operator= (const Logger&)=delete;
    
    static void packThreadLauncher(void *argv);
    static void writeThreadLauncher(void *argv);
    static void statsThreadLauncher(void *argv);
    
    /**
     * This thread packs queued data in buffers to optimize write throughput
     */
    void packThread();
    
    /**
     * This thread writes packed buffers to disk
     */
    void writeThread();
    
    /**
     * This thread prints stats
     */
    void statsThread();
    
    /**
     * Log logger stats using the logger itself
     */
    void logStats();
    
    static const unsigned int filenameMaxRetry=100;
    static const unsigned int maxRecordSize=256;
    static const unsigned int numFifoRecords=64;
    static const unsigned int bufferSize=32*1024; ///< Size of buffer
    static const unsigned int numBuffers=3;       ///< Number of buffers

    class Record
    {
    public:
        Record() : size(0) {}
        char data[maxRecordSize];
        unsigned int size;
    };
    
    class Buffer
    {
    public:
        Buffer() : size(0) {}
        char data[bufferSize];
        unsigned int size;
    };

    /// This is a FIFO buffer between senseThread() and packThread()
    miosix::Queue<Record,numFifoRecords> queuedSamples;

    std::list<Buffer *> fullList;  ///< Buffers between packThread() and writeThread()
    std::list<Buffer *> emptyList; ///< Buffers between packThread() and writeThread()
    miosix::FastMutex listHandlingMutex;   ///< To allow concurrent access to the lists
    miosix::ConditionVariable listWaiting; ///< To lock when buffers are all full/empty

    miosix::Thread *statsT;   ///< Thred printing stats
    miosix::Thread *writeT;   ///< Thread writing data to disk
    miosix::Thread *packT;    ///< Thread packing queued data in buffers
    
    volatile bool started=false;///< Logger is started and accepting data
    bool stopSensing=false;   ///< Signals threads to stop and terminate

    FILE *file;

    int statDroppedSamples=0; ///< Number of dropped sample due to fifo full
    int statWriteFailed=0;    ///< Number of fwrite() that failed
    int statWriteTime=0;      ///< Time to perform an fwrite() of a buffer
    int statMaxWriteTime=0;   ///< Max time to perform an fwrite() of a buffer
    int statQueuePush=0;      ///< Number of records successfully pushed to queue
    int statBufferFilled=0;   ///< Number of buffers filled
    int statBufferWritten=0;  ///< Number of buffers successfully written to disk
};
