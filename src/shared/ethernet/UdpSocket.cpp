/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Silvano Seva
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

#include "UdpSocket.h"
#include <string.h>

using namespace std;
using namespace miosix;

/** W5200 external interrupt IRQ handler **/

void __attribute__((naked)) EXTI1_IRQHandler() {
    saveContext();
    asm volatile("bl _Z13EXTIrqHandlerv");
    restoreContext();
}

void __attribute__((used)) EXTIrqHandler() {
    EXTI->PR |= EXTI_PR_PR1;
    UdpSocket::irqHandler();
}

Thread *waiting[8] = {0};

/* Class methods implementation */
uint8_t UdpSocket::instanceIndex = 0;
bool UdpSocket::initialized = false;
uint8_t UdpSocket::interruptFlags[8];    
W5200& UdpSocket::w5200 = W5200::instance();

UdpSocket::UdpSocket(const uint16_t& sockPort) {
    if(!initialized)
        init();
    
    bool emptyFound = false;
    
    for(uint8_t count = 0; count < maxSockNum && !emptyFound; count++) {
        if(!(instanceIndex & (0x01 << count))) {
            emptyFound = true;
            instanceIndex |= 0x01 << count;
            sockn = count;            
        }
    }

    if(!emptyFound) {
        // FIXME: No I/O here.
        puts("[Udp Socket] ERROR! No more sockets can be created!!");
    } else {
        w5200.setSocketModeReg(sockn,SOCKn_MR_UDP);
        w5200.setSocketSourcePort(sockn,sockPort);
        w5200.setSocketInterruptMaskReg(sockn,0xFF);
        w5200.setSocketCommandReg(sockn,SOCKn_CR_OPEN);
    }
}

UdpSocket::~UdpSocket() {
    w5200.setSocketModeReg(sockn,SOCKn_MR_CLOSE);
    instanceIndex &= ~(0x01 << sockn);
}

void UdpSocket::init() {
    eth::int1::mode(Mode::INPUT);    
    
    // Configure STM32 to generate an interrupt on falling edge 
    // of chip's INT line
    {
        FastInterruptDisableLock dLock;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
        RCC_SYNC();
    }
    
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;
    EXTI->IMR |= EXTI_IMR_MR1;
    EXTI->FTSR |= EXTI_FTSR_TR1;
    
    NVIC_SetPriority(EXTI1_IRQn,10);
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    
    w5200.setModeReg(0x00);
    w5200.setSocketInterruptMask(0xFF);
    
    initialized = true;
}

bool UdpSocket::sendTo(const uint8_t* destIp, const uint16_t destPort, const
                       void* data, uint16_t len) {
    w5200.setSocketDestIp(sockn,destIp);
    w5200.setSocketDestPort(sockn,destPort);
    w5200.writeData(sockn,reinterpret_cast<const uint8_t*>(data),len);
    w5200.setSocketCommandReg(sockn,SOCKn_CR_SEND);
    
    waiting[sockn] = Thread::getCurrentThread();
    {
        FastInterruptDisableLock dLock;
        while(waiting[sockn] && !(interruptFlags[sockn] & 0x18)) {
            waiting[sockn]->IRQwait();
            {
                FastInterruptEnableLock eLock(dLock);
                Thread::yield();
            }
        }
    }
    
    // The 4-th bit of socket interrupt register, if set, signals send failed
    if(interruptFlags[sockn] & 0x08) {
        interruptFlags[sockn] &= ~0x08;
        return false;        
    }
    
    // Clear both send ok and send fail flags
    interruptFlags[sockn] &= ~0x18;         
    return true;
}

uint16_t UdpSocket::receive(uint8_t* sourceIp, uint16_t *sourcePort, 
                            void* data) {
    w5200.setSocketCommandReg(sockn,SOCKn_CR_RECV);
    
    waiting[sockn] = Thread::getCurrentThread();
    {
        FastInterruptDisableLock dLock;
        while(waiting[sockn] && !(interruptFlags[sockn] & 0x04)) {
            waiting[sockn]->IRQwait();
            {                
                FastInterruptEnableLock eLock(dLock);
                Thread::yield();
            }
        }
    }
    
    // Clear received data flag
    interruptFlags[sockn] &= ~0x04;

    // Get new packet len
    uint16_t len = w5200.getReceivedSize(sockn);  
    uint8_t buffer[len];
    
    /* Read all the packet, that is made of sourceIp + sourcePort +
     * payload len + payload. Since the header length is of 8 bytes,
     * the payload length is len - 8 bytes! :)
     */
    w5200.readData(sockn,buffer,len);    

    memcpy(sourceIp,buffer,4);
    *sourcePort = (buffer[4] << 8) | buffer[5];
    memcpy(reinterpret_cast<uint8_t*>(data),buffer + 8,len - 8);
    
    return len - 8;
}


void UdpSocket::irqHandler()
{
    /* Read socket interrupt register. In this register is signalled
     * which of the sockets has rised an interrupt, in this way: if the
     * n-th bit is set the n-th socket has rised and interrupt.
     */
    
    uint8_t sockGlobalInt = w5200.readSocketInterruptReg();
    bool hasLowerPriority = false;

    for(uint8_t i = 0; i < 8; i++) {        
        if(sockGlobalInt & (0x01 << i)) {    
            // Read single socket interrupt register
            interruptFlags[i] = w5200.getSocketInterruptReg(i);
            w5200.clearSocketInterruptReg(i);
            if(waiting[i]) {
                waiting[i]->IRQwakeup();
                waiting[i] = 0;
                
                if(waiting[i]->IRQgetPriority() > 
                        Thread::IRQgetCurrentThread()->IRQgetPriority())
                    hasLowerPriority = true;
            }
        }
    }
    
    sockGlobalInt = 0;
    
    if(hasLowerPriority)
        Scheduler::IRQfindNextThread();    
}
