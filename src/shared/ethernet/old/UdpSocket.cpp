/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (c) 2016 Skyward Experimental Rocketry
 * Author: Silvano Seva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "UdpSocket.h"
#include <string.h>

using namespace std;
using namespace miosix;

/** W5200 external interrupt IRQ handler **/

void __attribute__((naked))  EXTI1_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z13EXTIrqHandlerv");
    restoreContext();
}

void __attribute__((used))  EXTIrqHandler()
{
    EXTI->PR |= EXTI_PR_PR1;
    UdpSocket::irqHandler();
}

Thread *waiting[8] = {0};

/** class methods implementation **/

uint8_t UdpSocket::instanceIndex = 0;
bool UdpSocket::initialized = false;
uint8_t UdpSocket::interruptFlags[8];    
W5200& UdpSocket::w5200 = W5200::instance();


UdpSocket::UdpSocket(const uint16_t& sockPort)
{
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
        puts("[Udp Socket] ERROR! No more sockets can be created!!");
    }else {        
        w5200.setSocketModeReg(sockn,SOCKn_MR_UDP);
        w5200.setSocketSourcePort(sockn,sockPort);
        w5200.setSocketInterruptMaskReg(sockn,0xFF);
        w5200.setSocketCommandReg(sockn,SOCKn_CR_OPEN);
    }
}

UdpSocket::~UdpSocket()
{
    w5200.setSocketModeReg(sockn,SOCKn_MR_CLOSE);
    instanceIndex &= ~(0x01 << sockn);
}

void UdpSocket::init()
{
    eth::int1::mode(Mode::INPUT);    
    
    //configure STM32 to generate an interrupt on falling edge of chip's INT line
    
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


bool UdpSocket::sendTo(const uint8_t* destIp, const uint16_t destPort, const void* data, uint16_t len)
{
    w5200.setSocketDestIp(sockn,destIp);
    w5200.setSocketDestPort(sockn,destPort);
    w5200.writeData(sockn,reinterpret_cast<const uint8_t*>(data),len);
    w5200.setSocketCommandReg(sockn,SOCKn_CR_SEND);
    
    waiting[sockn] = Thread::getCurrentThread();
    {
        FastInterruptDisableLock dLock;
        while(waiting[sockn] && !(interruptFlags[sockn] & 0x18))
        {
            waiting[sockn]->IRQwait();
            {
                FastInterruptEnableLock eLock(dLock);
                Thread::yield();
            }
        }
    }
    
    //the 4-th bit of socket interrupt register, if set, signals send failed
    if(interruptFlags[sockn] & 0x08) {
        interruptFlags[sockn] &= ~0x08;
        return false;        
    }
    
    //clear both send ok and send fail flags
    interruptFlags[sockn] &= ~0x18;         
    return true;
}

uint16_t UdpSocket::receive(uint8_t* sourceIp, uint16_t *sourcePort, void* data)
{
    w5200.setSocketCommandReg(sockn,SOCKn_CR_RECV);
    
    waiting[sockn] = Thread::getCurrentThread();
    {
        FastInterruptDisableLock dLock;
        while(waiting[sockn] && !(interruptFlags[sockn] & 0x04))
        {
            waiting[sockn]->IRQwait();
            {                
                FastInterruptEnableLock eLock(dLock);
                Thread::yield();
            }
        }
    }
    
    //clear received data flag
    interruptFlags[sockn] &= ~0x04;

    //get new packet len
    uint16_t len = w5200.getReceivedSize(sockn);  
    uint8_t buffer[len];
    
    //read all the packet, that is made of sourceIp + sourcePort +
    //payload len + payload. Since the header length is of 8 bytes,
    //the payload length is len - 8 bytes! :)
    w5200.readData(sockn,buffer,len);    

    memcpy(sourceIp,buffer,4);
    *sourcePort = (buffer[4] << 8) | buffer[5];
    memcpy(reinterpret_cast<uint8_t*>(data),buffer + 8,len - 8);
    
    return len - 8;
}


void UdpSocket::irqHandler()
{
    //read socket interrupt register. In this register is signalled
    //which of the sockets has rised an interrupt, in this way: if the
    //n-th bit is set the n-th socket has rised and interrupt
    
    uint8_t sockGlobalInt = w5200.readSocketInterruptReg();
    bool hasLowerPriority = false;

    for(uint8_t i = 0; i < 8; i++) {        
        if(sockGlobalInt & (0x01 << i)) {    
            
             //read single socket interrupt register
            interruptFlags[i] = w5200.getSocketInterruptReg(i);
            w5200.clearSocketInterruptReg(i);
            if(waiting[i]) {
                waiting[i]->IRQwakeup();
                waiting[i] = 0;
                
                if(waiting[i]->IRQgetPriority() > Thread::IRQgetCurrentThread()->IRQgetPriority())
                    hasLowerPriority = true;
            }
        }
    }
    
    sockGlobalInt = 0;
    
    if(hasLowerPriority)
        Scheduler::IRQfindNextThread();    
}



