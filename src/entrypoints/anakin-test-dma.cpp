/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
 * Authors: Alain Carlucci
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Common.h>
#include <drivers/Leds.h>
#include <boards/AnakinBoard.h>
#include <diagnostic/Log.h>
#include <diagnostic/CpuMeter.h>

using namespace miosix;

void fifoQueueSz(void *arg)
{
    const SPIDriver& spi = SPIDriver::instance();
    int tx_accum = 0, rx_accum = 0, sz_ctr = 0;
    int qsize_accum = 0;

    sLog->logString("Thread started");

    while(1)
    {
        DMAFIFOStatus tx = spi.getTxFIFOStatus();
        DMAFIFOStatus rx = spi.getRxFIFOStatus();

        if(tx > -1 && rx > -1)
        {
            tx_accum += tx;
            rx_accum += rx;
            qsize_accum += sLog->getLogQueueSize();
            if(++sz_ctr == 100)
            {
                float tx1 = tx_accum / (float)(DFS_100 - DFS_EE + 1) * 255.0f;
                float rx1 = rx_accum / (float)(DFS_100 - DFS_EE + 1) * 255.0f;
                float qsz = qsize_accum / 100.0f * 255.0f;
                tx_accum = rx_accum = sz_ctr = qsize_accum = 0;
                sLog->logLimitedInt(17, 0, 255, tx1);
                sLog->logLimitedInt(18, 0, 255, rx1);
                sLog->logUInt32(19, spi.getFIFOFaultCtr());
                sLog->logUInt32(20, averageCpuUtilization());
                sLog->logLimitedInt(21, 0, 255, qsz);
            }
        }
        Thread::sleep(1);
    }
}

int main()
{
    printf("\n");
    Leds::set(0);
    Log::getInstance();
    sBoard->init();

    const std::vector<SingleSensor>& data = sBoard->debugGetSensors();
    int ctr=0;

    Thread::create(fifoQueueSz, 1024);
    while(1)
    {
        for(const auto& s : data)
        {
            switch(s.data)
            {
                case DATA_VEC3:
                    sLog->logSensorVec3(s.sensor, *(Vec3 *)s.value);
                    break;
                case DATA_FLOAT:
                    sLog->logSensorFloat(s.sensor, *(float *)s.value);
                    break;
                default:
                    break;
            }
        }
    
        Thread::sleep(100);
    }

    return 0;
}
