/* CAN-Bus Driver
 *
 * Copyright (c) 2015 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci
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

/*
TODO:
- rimozione filtri quando un socket viene chiuso
- interrupt CAN2
- gestione automatica filtri a 32 bit
- gestione automatica filtri con il mascheramento

*/

#include "CanBus.h"
#include "CanSocket.h"
#include "CanUtils.h"

// Transmit mailbox request
#define TMIDxR_TXRQ       ((uint32_t)0x00000001) 


typedef std::multimap<int,CanSocket *>::iterator iterator;


CanBus::CanBus(CAN_TypeDef* CanStruct){
    CANx = CanStruct;
    terminate=false;
    pthread_create(&t,NULL,threadLauncher,reinterpret_cast<void*>(this));
}

// Bind a socket and set proper filters
bool CanBus::registerSocket(CanSocket *socket, uint16_t id) {
    if(id >= CanManager::filter_max_id)
        return false;

    //Lock mutex, add to multimap
    {
        Lock<FastMutex> l(mutex);
        if(!manager.addHWFilter(id, this->id)) {
            // TODO: log error 
            return false;
        }
        messageConsumers.insert(std::pair<int,CanSocket *>(id, socket));
    }

    return true;
}

// Unbind a socket and unset filters
bool CanBus::unregisterSocket(CanSocket *socket, uint16_t id) {
    if(id >= CanManager::filter_max_id)
        return false;

    //Lock mutex, remove from multimap, remove from filter bank
    {
        bool found = false;
        Lock<FastMutex> l(mutex);

        std::pair<iterator, iterator> ip = 
            messageConsumers.equal_range(id);

        for (auto& it = ip.first; it != ip.second; ++it) {
            if (it->second == socket) {
                messageConsumers.erase(it);
                found = true;
                break;
            }
        }

        if(!found)
            return false;

        if(!delHWFilter(id, this->id)) {
            // log unconsistency error
            return false;
        }
    }
}

/**
  funzione eseguita dal thread che smista i messaggi 
  ricevuti tra i vari receiver
  */
void CanBus::queueHandler(){
    while(terminate==false)
    {
        while(!messageQueue.isEmpty()){
            CanMsg message;

            messageQueue.get(message);

            dispatchMessage(message);
        }
    }
}

//metodo per eseguire queueHandler//
void* CanBus::threadLauncher(void* arg)
{
    reinterpret_cast<CanBus*>(arg)->queueHandler();
    return NULL;
}


/*
   invia un messaggio ad un determinato id di destinazione
   \param id identificativo del destinatario (Standard o Esteso)
   \param message il messaggio da inviare
   \param size la dimensione del messaggio
   */
bool CanBus::sendMessage(uint8_t id, 
        const unsigned char *message, size_t size) {
    int txMailBox = -1; 
    CanMsg packet;

    if(size == 0)
        return false;

    packet.StdId = id;
    packet.DLC = size <= 8 ? size : 8;

    memcpy(packet.Data[i], message[i], packet.DLC);

    {
        Lock<FastMutex> lock;

        int timeout = 10;
        while(txMailBox < 0 && timeout > 0){ 
            if ((CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
                txMailBox = 0;
            else if ((CANx->TSR & CAN_TSR_TME1) == CAN_TSR_TME1)
                txMailBox = 1;
            else if ((CANx->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
                txMailBox = 2;

            if(txMailBox < 0) {
                Thread::sleep(1);
                --timeout;
            }
        }

        if(txMailBox < 0)
            return false;

        CANx->sTxMailBox[txMailBox].TIR &= TMIDxR_TXRQ;
        CANx->sTxMailBox[txMailBox].TIR |= ((id << 21));

        // Make sure DLC is 4 bits only
        packet.DLC &= (uint8_t)0x0000000F;
        
        // Clean up size
        CANx->sTxMailBox[txMailBox].TDTR &= (uint32_t)0xFFFFFFF0; 
        CANx->sTxMailBox[txMailBox].TDTR |= packet.DLC;

        // Copy message data
        CANx->sTxMailBox[txMailBox].TDLR = (
                ((uint32_t)packet.Data[3] << 24) | 
                ((uint32_t)packet.Data[2] << 16) |
                ((uint32_t)packet.Data[1] << 8) | 
                ((uint32_t)packet.Data[0])
        );

        CANx->sTxMailBox[txMailBox].TDHR = (
                ((uint32_t)packet.Data[7] << 24) | 
                ((uint32_t)packet.Data[6] << 16) |
                ((uint32_t)packet.Data[5] << 8) |
                ((uint32_t)packet.Data[4])
        );

        // Request transmission
        CANx->sTxMailBox[txMailBox].TIR |= TMIDxR_TXRQ;
    }
    return true;
}

/**
  smista il messaggio tra i vari receiver registrati ad un determinato id
  \param message il messaggio da smistare
*/
void CanBus::dispatchMessage(CanMsg message){
    uint32_t id;

    if(message.IDE == CAN_ID_STD)
        id = message.StdId;
    else
        id = message.ExtId;

    uint8_t data[8];

    if(message.DLC > 8) 
        message.DLC = 8;

    memcpy(data, message.Data, message.DLC);

    {
        Lock<FastMutex> l(mutex);
        pair<iterator,iterator> ip = messageConsumers.equal_range(id);
        for(auto& it : ip) {
            CanSocket* socket = it->second;
            socket->addToMessageList(data, message.DLC);
        }
    }
}


/**
   inizializza il l'interfaccia can scelta 
   TODO: rimuovere la struttura per togliere il codice di st
*/
void CanBus::canSetup() {

    /*!<FIFO Message Pending Interrupt Enable */
    CANx->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1; 

    //esco da un eventuale sleep mode
    CANx->MCR &= ~CAN_MCR_SLEEP;

    //richiedo di entrare in modalità initialization
    CANx->MCR |= CAN_MCR_INRQ;

    // controllo di essere entrato nella modalità di inizializzazione
    while (((CANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK));

    //setto i vari bit di configurazione come impostati nella struttura ricevuta

    /* Unset the time triggered communication mode */
    CANx->MCR &= ~CAN_MCR_TTCM;

    /* Unset the automatic bus-off management */
    CANx->MCR &= ~CAN_MCR_ABOM;

    /* Unset the automatic wake-up mode */
    CANx->MCR &= ~CAN_MCR_AWUM;

    /* Unset the no automatic retransmission */
    CANx->MCR &= ~CAN_MCR_NART;

    /* Set the receive FIFO locked mode */
    CANx->MCR &= ~CAN_MCR_RFLM;

    /* Set the transmit FIFO priority */
    CANx->MCR &= ~CAN_MCR_TXFP;

    /* quanta 1+6+7 = 14, 14 * 3 = 42, 42000000 / 42 = 1000000 */
    /* CAN Baudrate = 1Mbps (CAN clocked at 42 MHz) Prescale = 3 */
    /* Requires a clock with integer division into APB clock */

    //sincronization with jump (CAN_BRT SJW)
    uint8_t CAN_SJW = CAN_SJW_1tq; // 1+6+7 = 14, 1+14+6 = 21, 1+15+5 = 21

    //Bit Segment 1 = sample point (CAN_BRT TS1)
    //Bit Segment 2 = transmit point (CAN_BRT TS2)
    uint32_t CAN_BS1 = CAN_BS1_6tq;
    uint32_t CAN_BS2 = CAN_BS2_7tq;

    //can prescaler (CAN_BRT BRP)
    uint16_t CAN_Prescaler = 3; 
    //3 == 42000000 / (14 * 1000000); // quanta by baudrate

    //pagina 165
    //if(RCC->CFGR & (1<<15)) //check se il clock viene diviso
    //freq /= (1<<(((RCC->CFGR>>13)&0x3)+1); 
    //freq = 168Mhz e viene diviso per 1,2,4,8,16 in base ai bit 14 e 13

#ifdef LOOPBACK
    uint8_t CAN_Mode = CAN_LOOPBACK;
#else
    uint8_t CAN_Mode = CAN_NORMAL;
#endif
    // setto i bit di timing e la modalità
    CANx->BTR = (CAN_Mode << 30) | \
                (CAN_SJW  << 24) | \
                (CAN_BS1  << 16) | \
                (CAN_BS2  << 20) | \
                (CAN_Prescaler - 1);


    //pulisco il bit INRQ per uscire dall'inizializzazione
    CANx->MCR &= ~((uint32_t)CAN_MCR_INRQ);
}

void __attribute__((naked)) CAN1_RX0_IRQHandler() {
    saveContext();
    asm volatile("mov r0, #0");
    asm volatile("mov r1, #0");
    asm volatile("bl _Z18CAN_IRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN1_RX1_IRQHandler() {
    saveContext();
    asm volatile("mov r0, #0");
    asm volatile("mov r1, #1");
    asm volatile("bl _Z18CAN_IRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN2_RX0_IRQHandler() {
    saveContext();
    asm volatile("mov r0, #1");
    asm volatile("mov r1, #0");
    asm volatile("bl _Z18CAN_IRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN2_RX1_IRQHandler() {
    saveContext();
    asm volatile("mov r0, #1");
    asm volatile("mov r1, #1");
    asm volatile("bl _Z18CAN_IRQHandlerImplii");
    restoreContext();
}

void __attribute__((used)) CAN_IRQHandlerImpl(int can_dev, int fifo) {
    //per prima cosa "resetto" l'interrupt altrimenti entro in un ciclo
    //  CAN1->RF0R |= CAN_RF0R_RFOM0; non è lui xD
    
    can_dev &= 0x01;
    CAN_TypeDef *can = can_dev?CAN2:CAN1;

    CanBus *manager = managerInstance[can_dev];
    CanMsg RxMessage;

    RxMessage.IDE = (uint8_t)0x04 & can->sFIFOMailBox[fifo].RIR;
    if (RxMessage.IDE == CAN_ID_STD)
        RxMessage.StdId = (uint32_t)0x000007FF & 
            (can->sFIFOMailBox[fifo].RIR >> 21);
    else
        RxMessage.ExtId = (uint32_t)0x1FFFFFFF & 
            (can->sFIFOMailBox[fifo].RIR >> 3);

    RxMessage.RTR = (uint8_t)0x02 & can->sFIFOMailBox[fifo].RIR;
    RxMessage.DLC = (uint8_t)0x0F & can->sFIFOMailBox[fifo].RDTR;
    RxMessage.FMI = (uint8_t)0xFF & (can->sFIFOMailBox[fifo].RDTR >> 8);

    /* Get the data field */
    *((uint32_t*)RxMessage.Data)     = can->sFIFOMailBox[fifo].RDLR;
    *((uint32_t*)(RxMessage.Data+4)) = can->sFIFOMailBox[fifo].RDHR;

    /* Release FIFO0 */
    can->RF0R |= CAN_RF0R_RFOM0;

    bool hppw=false;

    manager->messageQueue.IRQput(RxMessage,hppw);

    if(hppw) 
        Scheduler::IRQfindNextThread();
}
