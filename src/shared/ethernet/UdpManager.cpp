/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2017  <copyright holder> <email>
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

#include "UdpManager.h"

using namespace std;
using namespace miosix;

void __attribute__((naked))  EXTI1_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z13EXTIrqHandlerv");
    restoreContext();
}

void __attribute__((used))  EXTIrqHandler()
{
    EXTI->PR |= EXTI_PR_PR1;
    udpComm->IRQ_handler(UdpManager::IRQ_PHY);    
}

void _evt_mgmt_thread(void *args) {
    
    Singleton<UdpManager>::getInstance()->evtQueue.run();
}

UdpManager::UdpManager() : rxPacketCounter(0) {
    
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
    
    phy.setModeReg(0x00);
    phy.setSocketInterruptMask(0xFF);
        
    txBuffer = new PacketBuffer(TX_BUF_SIZE);
    rxBuffer = new PacketBuffer(RX_BUF_SIZE);
    
    if(!txBuffer->isValid() || !rxBuffer->isValid()) {
        //TODO log!

    }
    else {
    
    //Finally start _evt_mgmt_thread
        Thread::create(_evt_mgmt_thread,1024);
    }
}

UdpManager::~UdpManager() {
    
    delete txBuffer;
    delete rxBuffer;
}

void UdpManager::setPort(uint16_t port) {
    
    //Open transmitting socket
    phy.setSocketModeReg(PHY_TX_SOCK_NUM,SOCKn_MR_UDP);
    phy.setSocketSourcePort(PHY_TX_SOCK_NUM,port);
    // Enable timeout & send ok interrupts
    phy.setSocketInterruptMaskReg(PHY_TX_SOCK_NUM,0x18);
    phy.setSocketCommandReg(PHY_TX_SOCK_NUM,SOCKn_CR_OPEN);
    
    //Open receiving socket
    phy.setSocketModeReg(PHY_RX_SOCK_NUM,SOCKn_MR_UDP);
    phy.setSocketSourcePort(PHY_RX_SOCK_NUM,port);
    // Enable recv interrupt only
    phy.setSocketInterruptMaskReg(PHY_RX_SOCK_NUM,0x04);
    phy.setSocketCommandReg(PHY_RX_SOCK_NUM,SOCKn_CR_OPEN);
    phy.setSocketCommandReg(PHY_RX_SOCK_NUM,SOCKn_CR_RECV);
}

bool UdpManager::newReceivedPackets() {
    return !rxBuffer->empty();
}

void UdpManager::sendPacketTo(const uint8_t* ip, const uint16_t port,
                                            const void* data, size_t len) {
    packet_header_t header;
    header.ipAddress = *(reinterpret_cast< const uint32_t* >(ip));
    header.port = port;
    header.payloadSize = len;
    
    bool wasEmpty = txBuffer->empty();
    txBuffer->push(header,reinterpret_cast< const uint8_t*>(data));
    
    if(wasEmpty) {                
        evtQueue.post(bind(&UdpManager::tx_handler,this));
    }
}

void UdpManager::IRQ_handler(uint8_t irqNum) {
    bool hppw = false;

    if(irqNum == IRQ_PHY) {
        
        uint8_t sockInt = phy.readSocketInterruptReg();
        
        // TX socket interrupts management
        if(sockInt & (0x01 << PHY_TX_SOCK_NUM)) {
            uint8_t txFlags = phy.getSocketInterruptReg(PHY_TX_SOCK_NUM);
            phy.clearSocketInterruptReg(PHY_TX_SOCK_NUM);
            
            // Send OK interrupt flag set
            if(txFlags & 0x10) {
                evtQueue.IRQpost(bind(&UdpManager::tx_end_handler,this),hppw);
            }
            
            // Timeout flag set, problems with ARP
            if(txFlags & 0x08) {
                evtQueue.IRQpost(bind(&UdpManager::timeout_handler,this),hppw);
            }
        }
        
        // RX socket interrupts management
        if(sockInt & (0x01 << PHY_RX_SOCK_NUM)) {
            uint8_t rxFlags = phy.getSocketInterruptReg(PHY_RX_SOCK_NUM);
            phy.clearSocketInterruptReg(PHY_RX_SOCK_NUM);
            
            if(rxFlags & 0x04) {
                evtQueue.IRQpost(bind(&UdpManager::rx_handler,this),hppw);                
            }
        }
        
    }
    else if(irqNum == IRQ_TMO) {
        
        //TODO Timer expired interrupt management
    }
    
    if(hppw) {
        Scheduler::IRQfindNextThread();
    }
}

void UdpManager::tx_handler() {
    
    if(txBuffer->empty()) {
        return;
    }
    
    puts("yolla!");

    packet_header_t header = txBuffer->getHeader();
    uint8_t *addr = reinterpret_cast< uint8_t* >(&header.ipAddress);
    
    uint8_t payload[header.payloadSize];
    txBuffer->getData(payload);
    txBuffer->popFront();
    
    phy.setSocketDestIp(PHY_TX_SOCK_NUM,addr);
    phy.setSocketDestPort(PHY_TX_SOCK_NUM,header.port);
    phy.writeData(PHY_TX_SOCK_NUM,payload,header.payloadSize);
    phy.setSocketCommandReg(PHY_TX_SOCK_NUM,SOCKn_CR_SEND);
}

void UdpManager::tx_end_handler() {
    
    if(txBuffer->empty()) {
        return;
    }
    
    puts("finish!");
    
    evtQueue.post(bind(&UdpManager::tx_handler,this));
}

void UdpManager::timeout_handler() {

    //TODO: send timeout error infos!

    evtQueue.post(bind(&UdpManager::tx_end_handler,this));    
}

void UdpManager::rx_handler() {

    //get new packet len, in bytes
    uint16_t len = phy.getReceivedSize(PHY_RX_SOCK_NUM);  
    uint8_t buffer[len];
    
    //read all the packet, that is made of sourceIp + sourcePort +
    //payload len + payload. Since the header length is of 8 bytes,
    //the payload length is len - 8 bytes! :)
    phy.readData(PHY_RX_SOCK_NUM,buffer,len);
    
    // Set back to listening mode
    phy.setSocketCommandReg(PHY_RX_SOCK_NUM,SOCKn_CR_RECV);

    packet_header_t header;    
    
    memcpy(reinterpret_cast< uint8_t* >(&(header.ipAddress)),buffer,4);
    header.port = (buffer[4] << 8) | buffer[5];
    header.payloadSize = len - 8;
    
    bool pushOk = rxBuffer->push(header,&buffer[8]);

    //TODO: log pushOk status
}
