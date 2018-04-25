
#include "LogBase.h"
#ifdef _MIOSIX
#include <miosix.h>
#else //_MIOSIX
#include <cxxabi.h>
#endif //_MIOSIX

using namespace std;

//
// class LogBase
//

#ifdef _MIOSIX
LogBase::LogBase() : timestamp(miosix::getTick()) {}
#else //_MIOSIX
LogBase::LogBase() : timestamp(0) {}
#endif //_MIOSIX

void LogBase::print(ostream& os) const
{
    #ifdef _MIOSIX
    os<<"type="<<typeid(*this).name()<<" timestamp="<<timestamp<<' ';
    #else //_MIOSIX
    const char *name=typeid(*this).name();
    int status;
    char *demangled=abi::__cxa_demangle(name,NULL,0,&status);
    if(status==0 && demangled) name=demangled;
    os<<"type="<<name<<" timestamp="<<timestamp<<' ';
    if(demangled) free(demangled);
    #endif //_MIOSIX
}

LogBase::~LogBase() {}

//
// class LogStats
//

LogStats::LogStats() {}

void LogStats::print(ostream& os) const
{
    LogBase::print(os);
    os<<"ls="<<statTooLargeSamples
      <<" ds="<<statDroppedSamples
      <<" qs="<<statQueuedSamples
      <<" bf="<<statBufferFilled
      <<" bw="<<statBufferWritten
      <<" wf="<<statWriteFailed
      <<" wt="<<statWriteTime
      <<" mwt="<<statMaxWriteTime;
}

CEREAL_REGISTER_TYPE(LogStats);
