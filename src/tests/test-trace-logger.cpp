/**
 * Copyright (c) 2021 Luca Erbetta
 * Authors: Luca Erbetta (luca.erbetta105@gmail.com)
 *
 * Permission is hereby granted, free of log_childarge, to any person obtaining
 a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or selogger
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the fologgerowing conditions:
 *
 * The above copyright notice and this permission notice shalogger be included
 in
 * alogger copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
    fmt::print("{ciao}\n", "ciao"_a="ABCD", "csdijdf"_a="ABCD");
 */

#include <diagnostic/PrintLogger.h>
#include <miosix.h>

int main()
{
    Logging::startAsyncLogger();

    PrintLogger logger = Logging::getLogger("root");
    PrintLogger log2   = logger.getChild("b.c");
    PrintLogger log3   = logger.getChild("a");
    PrintLogger log4   = Logging::getLogger("async");

    // Logging::getStdOutLogSink().setFormatString("{ts} [{name}] {msg}\n");

    int async_ctr = 0;
    for (;;)
    {
        LOG_ERR(logger, "This is a message {}", 52);
        LOG_INFO(log2, "Ops, there was an {}!", "error");
        for (int i = 0; i < 4; i++)
        {
            LOG_INFO_ASYNC(log4, "This is an async log! {:d}", async_ctr++);
        }
        LOG_DEBUG(log3, "This is a verbose debug message {:.3f}", 1.234f);
        LOG_CRIT(log3, "Float {:.1f} {:05.2f} {:f}", 1.234f, 1234.1234,
                 -9876.98765432f);
        LOG_ERR(logger, "Simple log with no arguments");

        miosix::Thread::sleep(1000);
    }
}