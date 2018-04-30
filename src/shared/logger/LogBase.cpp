/***************************************************************************
 *   Copyright (C) 2018 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/ 

#include "LogBase.h"
#ifdef _MIOSIX
#include <miosix.h>
#else  //_MIOSIX
#include <cxxabi.h>
#endif  //_MIOSIX

using namespace std;

#ifdef USE_CEREAL

//
// class LogBase
//

#ifdef _MIOSIX
LogBase::LogBase() : timestamp(miosix::getTick()) {}
#else   //_MIOSIX
LogBase::LogBase() : timestamp(0) {}
#endif  //_MIOSIX

void LogBase::print(ostream& os) const
{
#ifdef _MIOSIX
    os << "type=" << typeid(*this).name() << " timestamp=" << timestamp << ' ';
#else   //_MIOSIX
    const char* name = typeid(*this).name();
    int status;
    char* demangled = abi::__cxa_demangle(name, NULL, 0, &status);
    if (status == 0 && demangled)
        name = demangled;
    os << "type=" << name << " timestamp=" << timestamp << ' ';
    if (demangled)
        free(demangled);
#endif  //_MIOSIX
}

LogBase::~LogBase() {}

//
// class LogStats
//

LogStats::LogStats() {}

void LogStats::print(ostream& os) const
{
    LogBase::print(os);
    os << "ls=" << statTooLargeSamples << " ds=" << statDroppedSamples
       << " qs=" << statQueuedSamples << " bf=" << statBufferFilled
       << " bw=" << statBufferWritten << " wf=" << statWriteFailed
       << " wt=" << statWriteTime << " mwt=" << statMaxWriteTime;
}

CEREAL_REGISTER_TYPE(LogStats);

#else //USE_CEREAL

//
// class LogStats
//

LogStats::LogStats() {}

void LogStats::print(ostream& os) const
{
    os << "timestamp=" << timestamp
       << " ls=" << statTooLargeSamples << " ds=" << statDroppedSamples
       << " qs=" << statQueuedSamples << " bf=" << statBufferFilled
       << " bw=" << statBufferWritten << " wf=" << statWriteFailed
       << " wt=" << statWriteTime << " mwt=" << statMaxWriteTime;
}

#endif //USE_CEREAL
