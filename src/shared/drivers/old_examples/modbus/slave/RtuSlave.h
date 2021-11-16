/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#ifndef RTUSLAVE_H
#define RTUSLAVE_H

#include <memory>
#include "../../Common.h"
#include "../PDU.h"
#include "Timer.h"

class RtuSlave : public Singleton< RtuSlave >
{
    
friend class Singleton< RtuSlave >;
public:
    ~RtuSlave();
    
    void setBaud(uint32_t baud) throw();
            
    void sendReply(std::unique_ptr<PDU>& data, uint8_t fromInterface);
    
    bool newDataReceived(uint8_t interface);
    
    std::pair<uint8_t, std::unique_ptr<PDU>> readData(uint8_t fromInterface);
    
    void IRQSerial2Interrupt();
    void IRQSerial3Interrupt();
    void IRQTimerInterrupt();
    
private:
    RtuSlave();
    
    void timerInit();
    void setNewTimeout(uint8_t channel, uint8_t ticks);
    
    typedef struct
    {
        uint8_t rxBuffer[256];
        uint8_t rxWriteIndx;
        uint16_t lastEvent;
        bool rxInProgress;
    
        uint8_t txBuffer[256];
        uint8_t txReadIndx;
        uint8_t txSize;
    
    }interf_data;
    
    interf_data serial2_data;
    interf_data serial3_data;
    
    float timer_k;
    
    RtuSlave(const RtuSlave& other) = delete;
    RtuSlave& operator=(const RtuSlave& other) = delete;
    bool operator==(const RtuSlave& other) = delete;       
};

#endif // RTUSLAVE_H
