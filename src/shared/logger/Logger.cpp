
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
    
    //The boring part, start threads one by one and if they fail, undo
    //Perhaps excessive defensive programming as thread creation failure is
    //highly unlikely (only if ram is full)
    
    packT=Thread::create(packThreadLauncher,4096,1,this,Thread::JOINABLE);
    if(!packT)
    {
        fclose(file);
        throw runtime_error("Error creating pack thread");
    }
    
    writeT=Thread::create(writeThreadLauncher,4096,1,this,Thread::JOINABLE);
    if(!writeT)
    {
        fullQueue.put(nullptr); //Signal packThread to stop
        packT->join();
        //packThread has pushed a buffer and a nullptr to writeThread, remove it
        while(fullList.front()!=nullptr)
        {
            emptyList.push(fullList.front());
            fullList.pop();
        }
        fullList.pop(); //Remove nullptr
        fclose(file);
        throw runtime_error("Error creating write thread");
    }
    statsT=Thread::create(statsThreadLauncher,4096,1,this,Thread::JOINABLE);
    if(!statsT)
    {
        fullQueue.put(nullptr); //Signal packThread to stop
        packT->join();
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
    fullQueue.put(nullptr); //Signal packThread to stop
    packT->join();
    writeT->join();
    statsT->join();
    fclose(file);
}

LogResult Logger::log(const LogBase& lb)
{
    if(started==false) return LogResult::Ignored;
    
    Record *record=nullptr;
    {
        FastInterruptDisableLock dLock;
        //We disable interrupts because IRQget() is nonblocking, unlike get()
        if(emptyQueue.IRQget(record)==false)
        {
            s.statDroppedSamples++;
            return LogResult::Dropped;
        }
    }
    record->size=0;

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
        emptyQueue.put(record); //Don't lose the record
        throw;
    }
    
    ss.seekp(0,ios::end);
    unsigned int dataSize=ss.tellp();
    if(dataSize>maxRecordSize)
    {
        s.statTooLargeSamples++;
        emptyQueue.put(record); //Don't lose the record
        return LogResult::TooLarge;
    }
    ss.read(record->data,dataSize);
    record->size=dataSize;
    
    fullQueue.put(record);
    s.statQueuedSamples++;
    return LogResult::Queued;
}

Logger::Logger()
{
    //Allocate buffers and put them in the empty list
    for(unsigned int i=0;i<numBuffers;i++) emptyList.push(new Buffer);
    for(unsigned int i=0;i<numRecords;i++) emptyQueue.put(new Record);
}

void Logger::packThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->packThread();
}

void Logger::writeThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->writeThread();
}

void Logger::statsThreadLauncher(void* argv)
{
    reinterpret_cast<Logger*>(argv)->statsThread();
}

void Logger::packThread()
{
    /*
     * The first implementation of this class had the log() function write
     * directly the serialized data to the buffers. So, no Records nor
     * packThread existed. However, to be able to call log() concurrently
     * without messing up the buffer, a mutex was needed. Thus, if many
     * threads call log(), contention on that mutex would occur, serializing
     * accesses and slowing down the (potentially real-time) callers. For this
     * reason Records and the pack thread were added.
     * Now each log() works independently on its own Record, and log() accesses
     * can proceed in parallel.
     */
    try {
        Buffer *buffer=nullptr;
        for(;;)
        {
            {
                Lock<FastMutex> l(mutex);
                //Put back previous buffer (except first time)
                if(buffer)
                {
                    fullList.push(buffer);
                    cond.broadcast();
                    s.statBufferFilled++;
                }
                
                //Get a full buffer, wait if none is available
                while(emptyList.empty()) cond.wait(l);
                buffer=emptyList.front();
                emptyList.pop();
                buffer->size=0;
            }
            
            do {
                Record *record=nullptr;
                fullQueue.get(record);
                
                //When stop() is called, it pushes a nullptr signaling to stop
                if(record==nullptr)
                {
                    fullList.push(buffer);
                    fullList.push(nullptr); //Signal writeThread to stop
                    cond.broadcast();
                    s.statBufferFilled++;
                    return;
                }
                
                memcpy(buffer->data+buffer->size,record->data,record->size);
                buffer->size+=record->size;
                emptyQueue.put(record);
            } while(bufferSize-buffer->size>=maxRecordSize);
        }
    } catch(exception& e) {
        printf("Error: packThread failed due to an exception: %s\n",e.what());
    }
}

void Logger::writeThread()
{
    try {
        Buffer *buffer=nullptr;
        for(;;)
        {
            {
                Lock<FastMutex> l(mutex);
                //Put back previous buffer (except first time)
                if(buffer)
                {
                    emptyList.push(buffer);
                    cond.broadcast();
                }
                
                //Get a full buffer, wait if none is available
                while(fullList.empty()) cond.wait(l);
                buffer=fullList.front();
                fullList.pop();
            }
            
            //When packThread stops, it pushes a nullptr signaling to stop
            if(buffer==nullptr) return;
            
            //Write data to disk
            Timer timer;
            timer.start();
            ledOn();
            
            size_t result=fwrite(buffer->data,1,buffer->size,file);
            if(result!=buffer->size)
            {
                //If this fails and your board uses SDRAM,
                //define and increase OVERRIDE_SD_CLOCK_DIVIDER_MAX
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
            if(started==false) return;
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
