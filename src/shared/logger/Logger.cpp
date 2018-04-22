
#include <logger/Logger.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;
using namespace miosix;

void Logger::start()
{
    char filename[32];
    for(unsigned int i=0;i<filenameMaxRetry;i++)
    {
        sprintf(filename,"/sd/%02d.dat",i);
        struct stat st;
        if(stat(filename,&st)==0) continue; //File exists
        if(i==filenameMaxRetry-1)
            printf("Too many existing files, overwriting last\n");
    }
    file=fopen(filename,"a+");
    if(file==NULL)
    {
        printf("Error opening log file\n");
        return;
    }
    setbuf(file,NULL);
    //Allocate buffers and put them in the empty list
    for(unsigned int i=0;i<numBuffers;i++) emptyList.push_back(new Buffer);
    statsT=Thread::create(statsThreadLauncher,4096,1,this,Thread::JOINABLE);
    writeT=Thread::create(writeThreadLauncher,4096,1,this,Thread::JOINABLE);
    packT= Thread::create(packThreadLauncher, 4096,1,this,Thread::JOINABLE);
    //TODO: better error handling, check if threads created
    started=true;
}

void Logger::stop()
{
    started=false;
    stopSensing=true;
    logStats(); //NOTE: also wakes threads eventually locked, prevents deadlock
    statsT->join();
    writeT->join();
    packT->join();
    //Now threads are surely stopped
    while(emptyList.empty()==false)
    {
        delete emptyList.front();
        emptyList.pop_front();
    }
    while(fullList.empty()==false)
    {
        delete fullList.front();
        fullList.pop_front();
        //FIXME: don't just delete the buffer, write it to disk first
    }
    fclose(file);
    printf("File closed\n");
    stopSensing=false;
}

Logger::Logger()
{
    
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
    try {
//         for(;;)
//         {
//             //Get an empty buffer, wait if none is available
//             Record *buffer;
//             {
//                 Lock<FastMutex> l(listHandlingMutex);
//                 while(emptyList.empty())
//                 {
//                     if(stopSensing) return;
//                     listWaiting.wait(l);
//                 }
//                 buffer=emptyList.front();
//                 emptyList.pop_front();
//             }
//             
//             //Read from the fifo and fill the buffer
//             for(int i=0;i<recordPerBuffer;i++)
//             {
//                 //FIXME: last partially filled buffer isn't written to disk
//                 if(stopSensing)
//                 {
//                     delete[] buffer; //Don't forget to delete the buffer
//                     return;
//                 }
//                 queuedSamples.get(buffer[i]);
//             }
//             statBufferFilled++;
//             
//             //Pass the buffer to the writeThread()
//             {
//                 Lock<FastMutex> l(listHandlingMutex);
//                 fullList.push_back(buffer);
//                 listWaiting.broadcast();
//             }
//         }
    } catch(exception& e) {
        printf("Error: packThread failed due to an exception: %s\n",e.what());
    }
}

void Logger::writeThread()
{
    try {
        for(;;)
        {
            //Get a full buffer, wait if none is available
            Buffer *buffer;
            {
                Lock<FastMutex> l(listHandlingMutex);
                while(fullList.empty())
                {
                    if(stopSensing) return;
                    listWaiting.wait(l);
                }
                buffer=fullList.front();
                fullList.pop_front();
            }
            
            //Write data to disk
            Timer timer;
            timer.start();
            ledOn();
            if(fwrite(buffer->data,1,buffer->size,file)!=buffer->size)
                statWriteFailed++;
            else statBufferWritten++;
            buffer->size=0;
            ledOff();
            timer.stop();
            statWriteTime=timer.interval();
            statMaxWriteTime=max(statMaxWriteTime,statWriteTime);
            
            //Return the buffer to the packThread()
            {
                Lock<FastMutex> l(listHandlingMutex);
                emptyList.push_back(buffer);
                listWaiting.broadcast();
            }
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
            if(stopSensing) return;
            Thread::sleep(1000);
            printf("ds:%d wf:%d wt:%d mwt:%d qp:%d bf:%d bw:%d\n",
                statDroppedSamples,statWriteFailed,statWriteTime,
                statMaxWriteTime,statQueuePush,statBufferFilled,statBufferWritten);
        }
    } catch(exception& e) {
        printf("Error: statsThread failed due to an exception: %s\n",e.what());
    }
}

