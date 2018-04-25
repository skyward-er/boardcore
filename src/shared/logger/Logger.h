
#pragma once

#include <cstdio>
#include <queue>
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
     * \return an instance to the logger
     */
    static Logger& instance();
    
    /**
     * Blocking call. May take a long time.
     * 
     * Call this function to start the logger.
     * When this function returns, the logger is started, and subsequent calls
     * to log will actually log the data.
     * 
     * \throws runtime_error if the log could not be opened
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
    
    static void writeThreadLauncher(void *argv);
    static void statsThreadLauncher(void *argv);
    
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
    void logStats() { s.setTimestamp(miosix::getTick()); log(s); }
    
    static const unsigned int filenameMaxRetry=100; ///< Limit on new filename
    static const unsigned int maxDataSize=256;      ///< Limit on logged data
    static const unsigned int bufferSize=64*1024;   ///< Size of each buffer
    static const unsigned int numBuffers=8;         ///< Number of buffers
    
    class Buffer
    {
    public:
        Buffer() : size(0) {}
        char data[bufferSize];
        unsigned int size;
    };

    std::queue<Buffer *,std::list<Buffer *>> fullList;  ///< Full buffers
    std::queue<Buffer *,std::list<Buffer *>> emptyList; ///< Empty buffers
    miosix::Mutex mutex;  ///< To allow concurrent access to the queues
    miosix::Mutex mutex2; ///< To allow concurrent log
    miosix::ConditionVariable cond; ///< To lock when buffers are all empty
    
    Buffer *currentBuffer=nullptr; ///< Producer side current buffer

    miosix::Thread *writeT;   ///< Thread writing data to disk
    miosix::Thread *statsT;   ///< Thred printing stats
    
    volatile bool started=false;///< Logger is started and accepting data
    bool stopSensing=true;      ///< Signals threads to stop and terminate

    FILE *file; ///< Log file
    LogStats s; ///< Logger stats
};
