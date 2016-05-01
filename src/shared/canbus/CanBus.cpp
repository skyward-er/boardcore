/* CAN-Bus Driver
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
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
#include "CanManager.h"
#include "CanSocket.h"
#include "CanUtils.h"

// Transmit mailbox request
#define TMIDxR_TXRQ       ((uint32_t)0x00000001) 

CanBus::CanBus(CAN_TypeDef *bus, CanManager* manager, const int id) : 
    CANx(bus), manager(manager), id(id) {
    terminate=false;
    pthread_create(&t,NULL,threadLauncher,reinterpret_cast<void*>(this));
    this->canSetup();
}

// Bind a socket and set proper filters
bool CanBus::registerSocket(CanSocket *socket) {
    uint16_t filter_id = socket->getFilterId();

    if(filter_id >= CanManager::filter_max_id)
        return false;

    //Lock mutex, add to map
    {
        Lock<FastMutex> l(mutex);

        set<CanSocket *> &ids = socket_map[filter_id];
        if(ids.find(socket) != ids.end())
            return false;

        if(!manager->addHWFilter(filter_id, this->id)) {
            // TODO: log error 
            return false;
        }
       
        ids.insert(socket);
    }

    return true;
}

// Unbind a socket and unset filters
bool CanBus::unregisterSocket(CanSocket *socket) {
    uint16_t filter_id = socket->getFilterId();

    if(id >= CanManager::filter_max_id)
        return false;

    //Lock mutex, remove from map and from filter bank
    {
        Lock<FastMutex> l(mutex);

        set<CanSocket *> &ids = socket_map[filter_id];
        auto it = ids.find(socket);

        if(it == ids.end())
            return false;

        if(!manager->delHWFilter(filter_id, this->id)) {
            // log unconsistency error
            return false;
        }

        ids.erase(it);
    }

    return true;
}

/**
  funzione eseguita dal thread che smista i messaggi 
  ricevuti tra i vari receiver
  */
void CanBus::queueHandler(){
    while(terminate==false)
    {
        CanMsg message;
        messageQueue.waitUntilNotEmpty();

        messageQueue.get(message);
        dispatchMessage(message);
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
   \param len la dimensione del messaggio
   */
bool CanBus::send(uint16_t id, const uint8_t *message, uint8_t len) {
    CanMsg packet;

    if(len == 0)
        return false;

    if(id > CanManager::filter_max_id)
        return false;

    packet.StdId = id;
    packet.DLC = len <= 8 ? len : 8;

    memcpy(packet.Data, message, packet.DLC);

    {
        int txMailBox = -1; 
        Lock<FastMutex> l(mutex);

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
        packet.DLC &= (uint8_t)0x0F;
        
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
    uint32_t filter_id;

    if(message.IDE == CAN_ID_STD)
        filter_id = message.StdId;
    else
        filter_id = message.ExtId;

    if(filter_id >= (uint32_t)CanManager::filter_max_id) {
        // log unsupported message
        return;
    }

    if(message.DLC > 8) 
        message.DLC = 8;

    {
        Lock<FastMutex> l(mutex);

        set<CanSocket *> &ids = socket_map[filter_id];
       
        for(auto socket : ids)
            socket->addToMessageList(message.Data, message.DLC);
    }
}

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

    //TODO: automatic retransmission causes the same packet to be resent forever
    //until the intended recipient acks it. That is a bit too much, so we're
    //disabling it. Maybe add a config option to allow setting it back?
    /* Unset the no automatic retransmission */
    //CANx->MCR &= ~CAN_MCR_NART;
    CANx->MCR |= CAN_MCR_NART;
    
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

