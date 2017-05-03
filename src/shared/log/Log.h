#ifndef SHARED_LOG_H
#define SHARED_LOG_H

#include <Common.h>
#include <Singleton.h>
#include <ActiveObject.h>
#include <Leds.h>
#include <queue>
#include <math/Vec3.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class Log : public Singleton<Log>, ActiveObject
{
    enum DataType
    {
        DATA_VEC3   = 0,
        DATA_QUAT   = 1,
        DATA_FLOAT  = 2,
        DATA_INT    = 3,
        DATA_STRING = 4,
        DATA_LIMITED_INT = 5,
    };

    friend class Singleton<Log>;
public:
    ~Log()
    {
    
    }

    void logString(const std::string& str)
    {
        std::vector<uint8_t> buf(1 + str.length());
        buf[0] = DATA_STRING; 
        for(size_t i=0;i<str.length();i++)
            buf[1+i] = str[i];

        queue(std::move(buf));
    }

    void logSensorFloat(uint8_t id, float data)
    {
        std::vector<uint8_t> buf(2 + sizeof(float));
        buf[0] = DATA_FLOAT;
        buf[1] = id;
        *((float *)&buf[2]) = data;

        queue(std::move(buf));
    }

    void logLimitedInt(uint8_t id, int min, int max, int data)
    {
        assert(max > min);
        uint32_t v = (data - min) * 256 / (max - min); 

        std::vector<uint8_t> buf(2 + 1);
        buf[0] = DATA_LIMITED_INT;
        buf[1] = id;
        buf[2] = v & 0xff;

        queue(std::move(buf));
    }

    void logSensorVec3(uint8_t id, const Vec3& data)
    {
        float tmp;
        std::vector<uint8_t> buf(2 + 3*sizeof(float));
        buf[0] = DATA_VEC3;
        buf[1] = id;

        tmp = data.getX(); *((float *)&buf[2]) = tmp;
        tmp = data.getY(); *((float *)&buf[6]) = tmp;
        tmp = data.getZ(); *((float *)&buf[10]) = tmp;

        queue(std::move(buf));
    }
protected:
    void run() override 
    {
        miosix::Lock<miosix::FastMutex> lock(mMutex);
        while(1)
        {
            Leds::set(4,1);
            while(mQueue.empty())
                mCondVar.wait(mMutex);
            
            Leds::set(4,0);
            while(!mQueue.empty())
            {
                const std::vector<uint8_t>& el = mQueue.front();
                write(el); 
                mQueue.pop();
            }
            mWaitVar.broadcast();
        }
    }
private:
    std::queue<std::vector<uint8_t>> mQueue;
    miosix::FastMutex mMutex;
    miosix::ConditionVariable mCondVar, mWaitVar;

    Log() : ActiveObject(1024)
    {
        /*
        struct termios t;
        tcgetattr(STDIN_FILENO, &t);
        t.c_lflag &= ~(ISIG | ICANON | ECHO);
        tcsetattr(STDIN_FILENO,TCSANOW, &t);
        */
    }

    void write(const std::vector<uint8_t>& data)
    {
        static const char map[]="0123456789abcdef";
        for(size_t i=0;i<data.size();i++)
        {
            const char& di = data[i];
            char out[2];
            out[0] = map[(di >> 4) & 0x0f];
            out[1] = map[di & 0x0f];
            ::write(1, out, 2);
        } 
        ::write(1, "\r\n", 2);
    }

    void queue(std::vector<uint8_t>&& data)
    {
        miosix::Lock<miosix::FastMutex> lock(mMutex);
        while(mQueue.size() > 100)
            mWaitVar.wait(lock);
        mQueue.push(std::move(data)); 
        mCondVar.signal();
    }
};

#define sLog Log::getInstance()

#endif /* ifndef SHARED_LOG_H */
