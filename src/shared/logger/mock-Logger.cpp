/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Federico Terraneo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <diagnostic/SkywardStack.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <errno.h>
#include <fcntl.h>
#include <interfaces/atomic_ops.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tscpp/buffer.h>
#include <utils/Debug.h>

#include <fstream>
#include <stdexcept>

#include "Logger.h"

using namespace std;
using namespace miosix;

namespace Boardcore
{

bool Logger::start() { return true; }

void Logger::stop() { return; }

bool Logger::testSDCard() { return true; }

int Logger::getCurrentLogNumber() { return -1; }

string Logger::getCurrentFileName() { return string(); }

LoggerStats Logger::getStats() { return LoggerStats{}; }

void resetStats() { return; }

bool Logger::isStarted() const { return true; }

void Logger::logStats() { return; }

LoggerResult Logger::logImpl(const char* name, const void* data,
                             unsigned int size)
{
    return LoggerResult::Ignored;
}

Logger::Logger() {}

string Logger::getFileName(int logNumber) { return string(""); }

void Logger::packThreadLauncher(void* argv) {}

void Logger::writeThreadLauncher(void* argv) {}

void Logger::packThread() {}

void Logger::writeThread() {}

}  // namespace Boardcore
