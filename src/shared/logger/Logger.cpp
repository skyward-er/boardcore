
#include "Logger.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>

using namespace std;
using namespace miosix;

//
// class Logger
//

Logger& Logger::instance()
{
    static Logger logger;
    return logger;
}

void Logger::start()
{
    if(started) return;
    stopSensing=false;
    
    char filename[32];
    for(unsigned int i=0;i<filenameMaxRetry;i++)
    {
        sprintf(filename,"/sd/%02d.dat",i);
        struct stat st;
        if(stat(filename,&st)!=0) break;
        //File exists
        if(i==filenameMaxRetry-1) puts("Too many files, appending to last");
    }
    
    file=fopen(filename,"ab");
    if(file==NULL) throw runtime_error("Error opening log file");
    setbuf(file,NULL);
    
    writeT=Thread::create(writeThreadLauncher,4096,1,this,Thread::JOINABLE);
    if(!writeT)
    {
        fclose(file);
        throw runtime_error("Error creating write thread");
    }
    statsT=Thread::create(statsThreadLauncher,4096,1,this,Thread::JOINABLE);
    if(!statsT)
    {
        stopSensing=true;
        writeT->join();
        fclose(file);
        throw runtime_error("Error creating stats thread");
    }
    started=true;
}

void Logger::stop()
{
    if(started==false) return;
    logStats();
    started=false;
    {
        //We lock mutex2 to be sure no other caller is still executing a log()
        Lock<Mutex> l2(mutex2);
        Lock<Mutex> l(mutex);
        if(currentBuffer)
        {
            fullList.push(currentBuffer);
            s.statBufferFilled++;
            currentBuffer=nullptr;
        }
    }
    stopSensing=true;
    cond.broadcast();
    writeT->join();
    statsT->join();
    fclose(file);
}

LogResult Logger::log(const LogBase& lb)
{
    //TODO: due to using a mutex log() may block, evaluate other sync primitives
    Lock<Mutex> l2(mutex2);
    if(started==false) return LogResult::Ignored;
    
    //First, make sure we have a valid buffer
    if(currentBuffer==nullptr)
    {
        Lock<Mutex> l(mutex);
        if(emptyList.empty())
        {
            s.statDroppedSamples++;
            return LogResult::Dropped;
        }
        currentBuffer=emptyList.front();
        emptyList.pop();
        currentBuffer->size=0;
    }
    
    //TODO: to increase performance, make a custom ostream/streambuf that
    //writes dirctly to the buffer eliminating heap usage within stringstream
    stringstream ss;
    unique_ptr<const LogBase> up(&lb);
    try {
        cereal::PortableBinaryOutputArchive archive(ss);
        archive(up);
        up.release();
    } catch(...) {
        // We are using unique_ptr only because cereal requires it.
        // We are not the owner of the object and we must not delete it.
        // If an exception is thrown, however, unique_ptr deletes the object.
        // This may be one of the very few cases where a try/catch is used
        // *not* to delete an object.
        up.release();
        throw;
    }
    
    ss.seekp(0,ios::end);
    unsigned int dataSize=ss.tellp();
    if(dataSize>maxDataSize)
    {
        s.statTooLargeSamples++;
        return LogResult::TooLarge;
    }
    ss.read(currentBuffer->data+currentBuffer->size,dataSize);
    currentBuffer->size+=dataSize;
    
    //If there's not enough space, commit the buffer
    if(bufferSize-currentBuffer->size<maxDataSize)
    {
        Lock<Mutex> l(mutex);
        fullList.push(currentBuffer);
        cond.broadcast();
        currentBuffer=nullptr;
        s.statBufferFilled++;
        //Optimization: see if we can already find a buffer (saves a mutex lock)
        if(emptyList.empty()==false)
        {
            currentBuffer=emptyList.front();
            emptyList.pop();
            currentBuffer->size=0;
        }
    }
    
    s.statQueuedSamples++;
    return LogResult::Queued;
}

Logger::Logger()
{
    //Allocate buffers and put them in the empty list
    for(unsigned int i=0;i<numBuffers;i++) emptyList.push(new Buffer);
}

void Logger::writeThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->writeThread();
}

void Logger::statsThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->statsThread();
}

void Logger::writeThread()
{
    try {
        Buffer *buffer=nullptr;
        for(;;)
        {
            {
                Lock<Mutex> l(mutex);
                //Put back previous buffer (except first time)
                if(buffer) emptyList.push(buffer);
                
                //Get a full buffer, wait if none is available
                while(fullList.empty())
                {
                    if(stopSensing) return; //Stop only when all buffers written
                    cond.wait(l);
                }
                buffer=fullList.front();
                fullList.pop();
            }
            
            //Write data to disk
            Timer timer;
            timer.start();
            ledOn();
            
            size_t result=fwrite(buffer->data,1,buffer->size,file);
            if(result!=buffer->size)
            {
                //perror("fwrite");
                s.statWriteFailed++;
            } else s.statBufferWritten++;
            ledOff();
            timer.stop();
            s.statWriteTime=timer.interval();
            s.statMaxWriteTime=max(s.statMaxWriteTime,s.statWriteTime);
        }
    } catch(exception& e) {
        printf("Error: writeThread failed due to an exception: %s\n",e.what());
    }
}

/**
 * This thread prints stats
 */
void Logger::statsThread()
{
    try {
        for(;;)
        {
            Thread::sleep(1000);
            if(stopSensing) return;
            logStats();
            printf("ls:%d ds:%d qs:%d bf:%d bw:%d wf:%d wt:%d mwt:%d\n",
                s.statTooLargeSamples,s.statDroppedSamples,s.statQueuedSamples,
                s.statBufferFilled,s.statBufferWritten,s.statWriteFailed,
                s.statWriteTime,s.statMaxWriteTime);
        }
    } catch(exception& e) {
        printf("Error: statsThread failed due to an exception: %s\n",e.what());
    }
}
