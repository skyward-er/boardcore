
#include "LogBase.h"
#include <miosix.h>

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
    os<<"type="<<typeid(this).name()<<" timestamp="<<timestamp<<' ';
}

LogBase::~LogBase() {}

//
// class LogStats
//

LogStats::LogStats() {}

void LogStats::print(ostream& os) const
{
    os<<"ls: "<<statTooLargeSamples
      <<"ds: "<<statDroppedSamples
      <<"qs: "<<statQueuedSamples
      <<"bf: "<<statBufferFilled
      <<"bw: "<<statBufferWritten
      <<"wf: "<<statWriteFailed
      <<"wt: "<<statWriteTime
      <<"mwt: "<<statMaxWriteTime;
}

CEREAL_REGISTER_TYPE(LogStats);
