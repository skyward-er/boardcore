
#include <cstdio>
#include <cstring>
#include <miosix.h>
#include <logger/Logger.h>
#include <diagnostic/CpuMeter.h>

using namespace std;
using namespace miosix;


class Dummy : public LogBase
{
public:
    Dummy() { memset(x,0,sizeof(x)); }
    
    void correctValue()
    {
        for(int i=0;i<num;i++) x[i]=42;
    }

    template<typename Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<LogBase>(this),x);
    }
    
    void print(std::ostream& os) const
    {
        LogBase::print(os);
        for(int i=0;i<num;i++)
        {
            if(x[i]==42) continue;
            os<<"unserialized incorrectly, x["<<i<<"]="<<x[i];
            return;
        }
        os<<"ok";
    }
private:
    static const int num=50;
    int x[num];
};

CEREAL_REGISTER_TYPE(Dummy);

void logthread(void*)
{
    Logger& log=Logger::instance();
    for(;;)
    {
        Thread::sleep(5);
        for(int i=0;i<5;i++)
        {
            Dummy d;
            d.correctValue();
            log.log(d);
        }
    }
}

void printutil(void*)
{
    for(;;)
    {
        Thread::sleep(1000);
        printf("cpu: %5.1f\n",averageCpuUtilization());
    }
}

int main()
{
    Thread::create(printutil,4096);
    
    Logger& log=Logger::instance();
    log.start();
    
    puts("type enter to start test");
    getchar();
    
    Thread::create(logthread,4096);
    
    puts("type enter to stop test");
    getchar();
    
    log.stop();
    
    puts("stopped");
    for(;;) { Thread::sleep(1000); }
}
