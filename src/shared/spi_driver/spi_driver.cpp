/***************************************************************************
 *   Copyright (C) 2016 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#include "spi_driver.h"
#include <kernel/scheduler/scheduler.h>

using namespace std;
using namespace miosix;

//Mapping for stm32f429 SPI5 (the one attached to mems)
typedef Gpio<GPIOF_BASE,8> miso;
typedef Gpio<GPIOF_BASE,9> mosi;
typedef Gpio<GPIOF_BASE,7> sck;

/**
 * DMA RX end of transfer
 */
void __attribute__((naked)) DMA2_Stream3_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20SPI5rxDmaHandlerImplv");
    restoreContext();
}

static Thread *waiting;
static vector<SPIRequest> *requestVector;
static size_t requestIndex;
static bool error;

/**
 * DMA RX end of transfer actual implementation
 */
void __attribute__((used)) SPI5rxDmaHandlerImpl()
{
    if(DMA2->LISR & (DMA_LISR_TEIF3 | DMA_LISR_DMEIF3)) error=true;
    DMA2->LIFCR=DMA_LIFCR_CTCIF3
              | DMA_LIFCR_CTEIF3
              | DMA_LIFCR_CDMEIF3;
    
    (*requestVector)[requestIndex].IRQendTransaction();
    if(++requestIndex>=requestVector->size())
    {
        waiting->IRQwakeup();
        if(waiting->IRQgetPriority()>Thread::IRQgetCurrentThread()->IRQgetPriority())
            Scheduler::IRQfindNextThread();
        waiting=0;
    } else {
        (*requestVector)[requestIndex].IRQbeginTransaction();
    }
}

//
// class SPIDriver
//

SPIDriver& SPIDriver::instance()
{
    static SPIDriver singleton;
    return singleton;
}

bool SPIDriver::transaction(vector<SPIRequest>& requests)
{
    if(requests.empty()) return false;
    for(auto req : requests) if(req.empty()) return false;
    
    pthread_mutex_lock(&mutex);
    error=false;
    
    waiting=Thread::getCurrentThread();  
    requestIndex=0;
    requestVector=&requests;
    
    {
        FastInterruptDisableLock dLock;
        
        (*requestVector)[requestIndex].IRQbeginTransaction();
        
        while(waiting!=0)
        {
            waiting->IRQwait();
            {
                FastInterruptEnableLock eLock(dLock);
                Thread::yield();
            }
        }
    }
   
    bool result=!error;
    pthread_mutex_unlock(&mutex);
    return result;
}

SPIDriver::SPIDriver()
{
    pthread_mutex_init(&mutex,0);
    {
        FastInterruptDisableLock dLock;
        mosi::mode(Mode::ALTERNATE);
        mosi::alternateFunction(5);
        miso::mode(Mode::ALTERNATE);
        miso::alternateFunction(5);
        sck::mode(Mode::ALTERNATE);
        sck::alternateFunction(5);
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
        SPI5->CR2=SPI_CR2_TXDMAEN
                | SPI_CR2_RXDMAEN;
        SPI5->CR1=SPI_CR1_SSM  //Software cs
                | SPI_CR1_SSI  //Hardware cs internally tied high
                | SPI_CR1_MSTR //Master mode
                | SPI_CR1_BR_0
                | SPI_CR1_BR_1 //Less than 10MHz
                | SPI_CR1_SPE; //SPI enabled
        NVIC_SetPriority(DMA2_Stream3_IRQn,10);//Low priority for DMA
        NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    }
}

//
// class SPIRequest
//

void SPIRequest::IRQbeginTransaction()
{
    chipSelect.low();
    
    DMA2_Stream3->CR=0;
    DMA2_Stream3->PAR=reinterpret_cast<unsigned int>(&SPI5->DR);
    DMA2_Stream3->M0AR=reinterpret_cast<unsigned int>(fromPeripheral.data());
    DMA2_Stream3->NDTR=fromPeripheral.size();
    DMA2_Stream3->CR=DMA_SxCR_CHSEL_1 //Channel 2
                   | DMA_SxCR_PL_1    //High priority because fifo disabled
                   | DMA_SxCR_MINC    //Increment memory pointer
                   | DMA_SxCR_TCIE    //Interrupt on transfer complete
                   | DMA_SxCR_TEIE    //Interrupt on transfer error
                   | DMA_SxCR_DMEIE   //Interrupt on direct mode error
                   | DMA_SxCR_EN;     //Start DMA
    
    DMA2_Stream4->CR=0;
    DMA2_Stream4->PAR=reinterpret_cast<unsigned int>(&SPI5->DR);
    DMA2_Stream4->M0AR=reinterpret_cast<unsigned int>(toPeripheral.data());
    DMA2_Stream4->NDTR=toPeripheral.size();
    DMA2_Stream4->CR=DMA_SxCR_CHSEL_1 //Channel 2
                   | DMA_SxCR_PL_1    //High priority because fifo disabled
                   | DMA_SxCR_MINC    //Increment memory pointer
                   | DMA_SxCR_DIR_0   //Memory to peripheral
                   | DMA_SxCR_TCIE    //Interrupt on transfer complete
                   | DMA_SxCR_TEIE    //Interrupt on transfer error
                   | DMA_SxCR_DMEIE   //Interrupt on direct mode error
                   | DMA_SxCR_EN;     //Start DMA
}
    
void SPIRequest::IRQendTransaction()
{
    //FIXME: ensure dead time between CS high and CS low!!!
    //And, is it really needed?
    chipSelect.high();
}
