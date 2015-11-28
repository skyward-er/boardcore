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

#define TMIDxR_TXRQ       ((uint32_t)0x00000001) /* Transmit mailbox request */

static const int numCanInterface=2; //CAN1, CAN2

typedef std::multimap<int,CanSocket *>::iterator iterator;

//array di puntatori alle classe per accedere all'oggetto dentro l'interrupt
CanBus* managerInstance[numCanInterface]={0}; 

CanBus::CanBus(CAN_TypeDef* CanStruct){
    CANx = CanStruct;
    terminate=false;
    pthread_create(&t,NULL,threadLauncher,reinterpret_cast<void*>(this));

    memset(filterMatrix,0,sizeof(filterMatrix));
    memset(availableMatrix,0,sizeof(availableMatrix));
}

CanBus* CanBus::getCanBus(CAN_TypeDef* CanStruct) {
    if(CanStruct==CAN1){
        if(managerInstance[0]==NULL){
            CanBus* manager = new CanBus(CanStruct);
            manager->canSetup();
            managerInstance[0]=manager;
        }
        return managerInstance[0];
    }


    if(CanStruct==CAN2){
        if(managerInstance[1]==NULL){
            CanBus* manager = new CanBus(CanStruct);
            manager->canSetup();
            managerInstance[1]=manager;
        }
        return managerInstance[1];
    }
    return NULL;

}


/** 
  registra nella propria mappa l'oggetto socket e imposta i filtri del can
  \param socket l'oggetto socket
  \param s l'id
  */
void CanBus::registerSocket(CanSocket *socket, uint8_t id)
{
    //Lock mutex, add to multimap
    {
        Lock<FastMutex> l(mutex);
        messageConsumers.insert(std::pair<int,CanSocket *>(id, socket));
        addToFilterBank(id);
    }

}
/** 
  rimuove  l'oggetto socket ed i relativi filtri 
  \param socket l'oggetto socket
  \param s l'id
  */
void CanBus::unregisterSocket(CanSocket *socket, uint8_t id)
{
    //Lock mutex, remove from multimap, remove from filter bank
    {
        Lock<FastMutex> l(mutex);

        std::pair<iterator, iterator> iteratorpair = messageConsumers.equal_range(id);

        for (iterator it = iteratorpair.first ; it != iteratorpair.second; ++it) {
            if (it->second == socket) {
                messageConsumers.erase(it);
                break;
            }
        }
    }
}

/**
  funzione eseguita dal thread che smista i messaggi ricevuti tra i vari receiver
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
void CanBus::sendMessage(uint8_t id, const unsigned char *message, int size){
    int timeout = 100;
    int txMailBox = -1; 

    CanMsg packet;

    packet.StdId = id;
    if(size<=8)
        packet.DLC=size;
    else
        packet.DLC=8;


    for(int i=0;i<size && i<8;i++){
        packet.Data[i]=(uint8_t)message[i];
    }

    /* aspetto finchè si libera una mailbox  */ 
    
    //è corretto? deve essere bloccante? 
    //ritorno null se dopo un tot non riesce ad inviare?
    while(txMailBox<0 && timeout >0){ 

        if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
        {
            txMailBox = 0;
        }
        else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
        {
            txMailBox = 1;
        }
        else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
        {
            txMailBox = 2;

        }
        timeout--;
    }

    CANx->sTxMailBox[txMailBox].TIR &= TMIDxR_TXRQ;

    //devo scrivere cose diverse nel registro CAN_TIxR in base al tipo di id
    if (CAN_ID_TYPE==CAN_ID_STD){
        CANx->sTxMailBox[txMailBox].TIR |= ((id << 21));
    }
    else{
        CANx->sTxMailBox[txMailBox].TIR |= ((id<< 3) | CAN_ID_EXT | CAN_TI0R_RTR);  
    }

    packet.DLC &= (uint8_t)0x0000000F; //mi assicuro siano solo 4 bit

    CANx->sTxMailBox[txMailBox].TDTR &= (uint32_t)0xFFFFFFF0; //azzero la size nel registro
    CANx->sTxMailBox[txMailBox].TDTR |= packet.DLC;

    //scrivo il dato nella parte bassa e alta del registro
    CANx->sTxMailBox[txMailBox].TDLR = (((uint32_t)packet.Data[3] << 24) | 
            ((uint32_t)packet.Data[2] << 16) |
            ((uint32_t)packet.Data[1] << 8) | 
            ((uint32_t)packet.Data[0]));

    CANx->sTxMailBox[txMailBox].TDHR = (((uint32_t)packet.Data[7] << 24) | 
            ((uint32_t)packet.Data[6] << 16) |
            ((uint32_t)packet.Data[5] << 8) |
            ((uint32_t)packet.Data[4]));


    //richiedo la trasmissione
    CANx->sTxMailBox[txMailBox].TIR |= TMIDxR_TXRQ;

    //dispatchMessage(id,message,size);
}

/**
  smista il messaggio tra i vari receiver registrati ad un determinato id
  \param message il messaggio da smistare
  */
void CanBus::dispatchMessage(CanMsg message){
    uint8_t id;

    if(message.IDE==CAN_ID_STD)
        id=message.StdId;
    else
        id=message.ExtId;

    uint8_t data[8];

    if(message.DLC > 8) message.DLC = 8;
    if(message.DLC < 0) message.DLC = 0;
    memcpy(data, message.Data, message.DLC);

    {
        Lock<FastMutex> l(mutex);
        pair<iterator, iterator> iteratorpair = messageConsumers.equal_range(id);
        for (iterator it = iteratorpair.first; it != iteratorpair.second; ++it) {
            CanSocket* socket = it->second;
            socket->addToMessageList(data,message.DLC);
        }
    }
}


/**
  inizializza il l'interfaccia can scelta 
TODO: rimuovere la struttura per togliere il codice di st
*/
void CanBus::canSetup(){
    CAN_ID_TYPE=CAN_ID_STD;

    if(CANx==CAN1){
        {
            FastInterruptDisableLock dLock;
            RCC->APB1ENR |= RCC_APB1ENR_CAN1EN; 
            RCC_SYNC();
        }
        NVIC_SetPriority(CAN1_RX0_IRQn,15); //Low priority   
        NVIC_EnableIRQ(CAN1_RX0_IRQn);
        NVIC_SetPriority(CAN1_RX1_IRQn,15); //Low priority   
        NVIC_EnableIRQ(CAN1_RX1_IRQn);
    }
    if(CANx==CAN2){
        {
            FastInterruptDisableLock dLock;
            RCC->APB1ENR |= RCC_APB1ENR_CAN1EN; 
            RCC->APB1ENR |= RCC_APB1ENR_CAN2EN; 
            RCC_SYNC();
        }
        NVIC_SetPriority(CAN2_RX0_IRQn,15); //Low priority   
        NVIC_EnableIRQ(CAN2_RX0_IRQn);
        NVIC_SetPriority(CAN2_RX1_IRQn,15); //Low priority   
        NVIC_EnableIRQ(CAN2_RX1_IRQn);
    }

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
    //freq /= (1<<(((RCC->CFGR>>13)&0x3)+1); //freq = 168Mhz e viene diviso per 1,2,4,8,16 in base ai bit 14 e 13

#ifdef LOOPBACK
    uint8_t CAN_Mode = CAN_LOOPBACK;
#else
    uint8_t CAN_Mode = CAN_NORMAL;
#endif
    /* setto i bit di timing e la modalità*/
    CANx->BTR = (CAN_Mode << 30) | \
                (CAN_SJW  << 24) | \
                (CAN_BS1  << 16) | \
                (CAN_BS2  << 20) | \
                (CAN_Prescaler - 1);


    //pulisco il bit INRQ per uscire dall'inizializzazione
    CANx->MCR &= ~((uint32_t)CAN_MCR_INRQ);

}




bool CanBus::addToFiltersMatrix(uint16_t filter,uint8_t* row) {
    uint8_t offset;

    offset = CANx==CAN1 ? FILTER_CAN1_INDEX : FILTER_CAN2_INDEX;

    for (int y=offset; y<FILTER_ROWS_PER_CAN+offset; y++)
        for (int x=0; x<FILTER_IDS_PER_ROW; x++)
            if(filterMatrix[y][x] == filter || availableMatrix[y][x]==0)
            {
                filterMatrix[y][x]=filter;
                availableMatrix[y][x]+=1;
                *row=y;
                return true;
            }
    //la matrice è piena
    return false;
}


void CanBus::addToFilterBank(uint8_t id){
    //TODO: seleziono automaticamente un filtro libero
    //mi segno i filtri che uso, la modalità di utilizzo e quanti spazi ho riempito per filtro
    //in modo da massimizzare il numero di id impostabili 
    //reminder: ovviamente non superare il max numero di filtri

    uint8_t row;
    if(!addToFiltersMatrix(id,&row)){
        printf("Errore matrice filtri piena\n");
        return;
    }

    ///////////////////
    uint32_t filtro=row; 
    uint32_t posizione_filtro = ((uint32_t)1) << filtro;


    uint32_t filterSlot[4];

    filterSlot[0]=filterMatrix[filtro][0]<<5;
    filterSlot[1]=filterMatrix[filtro][1]<<5;
    filterSlot[2]=filterMatrix[filtro][2]<<5;
    filterSlot[3]=filterMatrix[filtro][3]<<5;


    // uso CAN1 perchè i registri dei filtri sono condivisi tra CAN1 [0-13] e CAN2 [14-27]
    // http://www.developermemo.com/1609181/

    // attivo la modalità di inizializzazione per i filtri
    CAN1->FMR |= CAN_FMR_FINIT;

    //CAN1->FMR = (CAN1->FMR & 0xFFFF0000) | (14 << 8) | CAN_FMR_FINIT;





    //disattivo il filtro selezionato
    CAN1->FA1R &= ~(uint32_t)posizione_filtro;

    //0: Dual 16-bit scale configuration 1: Single 32-bit scale configuration
    //lo imposto a 2x16 bit
    //CAN1->FS1R &= ~(uint32_t)posizione_filtro;
    CAN1->FS1R |= posizione_filtro;

    /* 16-bit scale for the filter */
    CAN1->FS1R &= ~(uint32_t)posizione_filtro;

    /* First 16-bit identifier and First 16-bit mask */
    /* Or First 16-bit identifier and Second 16-bit identifier */
    CAN1->sFilterRegister[filtro].FR1 = 
        ((0x0000FFFF & (uint32_t)filterSlot[1]) << 16) |
        (0x0000FFFF & (uint32_t)filterSlot[0]);

    /* Second 16-bit identifier and Second 16-bit mask */
    /* Or Third 16-bit identifier and Fourth 16-bit identifier */
    CAN1->sFilterRegister[filtro].FR2 = 
        ((0x0000FFFF & filterSlot[3] << 16) |
         (0x0000FFFF & (uint32_t)filterSlot[2]));

    //modalità di utilizzo del filtro 0 mascheramento 1 lista di id
    // come IdList posso usare lo spazio della maschera per un altro id
    CAN1->FM1R |= (uint32_t)posizione_filtro;

    //assegnamento del filtro ad una fifo (0: FIFO0, 1: FIFO1)
    //  CAN1->FFA1R |= (uint32_t)posizione_filtro;
    CAN1->FFA1R &= ~(uint32_t)posizione_filtro;

    //attivo il filtro
    CAN1->FA1R |= posizione_filtro;

    //esco dalla modalità inizializzazione
    CAN1->FMR &= ~CAN_FMR_FINIT;
}

CanBus::~CanBus()
{
    if(CANx==CAN1)
        managerInstance[0]=NULL;
    if(CANx==CAN2)
        managerInstance[1]=NULL;
}

void __attribute__((naked)) CAN1_RX0_IRQHandler() {

    saveContext();
    asm volatile("mov r0, #0");
    asm volatile("mov r1, #0");
    asm volatile("bl _Z18CAN_IRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN1_RX1_IRQHandler() {
    const void *canN = (void *)CAN1; (void)canN;

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
        RxMessage.StdId = (uint32_t)0x000007FF & (can->sFIFOMailBox[fifo].RIR >> 21);
    else
        RxMessage.ExtId = (uint32_t)0x1FFFFFFF & (can->sFIFOMailBox[fifo].RIR >> 3);

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


//Interrupt CAN1 FIFO1
