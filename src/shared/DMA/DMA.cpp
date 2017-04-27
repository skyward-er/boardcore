/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
 * Authors: Federico Terraneo, Alain Carlucci
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

#include "DMA.h"
#include <kernel/scheduler/scheduler.h>

using namespace std;
using namespace miosix;

//Mapping for anakin board SPI1 (the one attached to mems)
typedef Gpio<GPIOA_BASE,5> sck;
typedef Gpio<GPIOA_BASE,6> miso;
typedef Gpio<GPIOA_BASE,7> mosi;

/**
 * DMA RX end of transfer
 */
void __attribute__((naked)) DMA2_Stream5_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20SPI1rxDmaHandlerImplv");
    restoreContext();
}

static Thread *waiting = NULL;
static vector<SPIRequest> *requestVector;
static size_t requestIndex = 0;
static bool error = false;

/**
 * DMA RX end of transfer actual implementation
 */
void __attribute__((used)) SPI1rxDmaHandlerImpl()
{
    if(DMA2->LISR & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0)) 
        error = true;
    DMA2->LIFCR = DMA_LIFCR_CTCIF0
                | DMA_LIFCR_CTEIF0
                | DMA_LIFCR_CDMEIF0;
    
    (*requestVector)[requestIndex].IRQendTransaction();

    if( ++requestIndex >= requestVector->size() )
    {
        waiting->IRQwakeup();
        if(waiting->IRQgetPriority() > 
                            Thread::IRQgetCurrentThread()->IRQgetPriority()) 
        {
            Scheduler::IRQfindNextThread();
        }

        waiting=0;
    } 
    else
    {
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
    for(auto& req : requests) if(req.empty()) return false;
    
    pthread_mutex_lock(&mutex);
    error=false;
    enableDMA();
    
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
    disableDMA();
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
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        disableDMA();
        SPI1->CR1 = SPI_CR1_SSM  //Software cs
                  | SPI_CR1_SSI  //Hardware cs internally tied high
                  | SPI_CR1_MSTR //Master mode
//                  | SPI_CR1_BR_0
//                  | SPI_CR1_BR_1 //Less than 10MHz
                  | SPI_CR1_BR_2 // fClock / 32
                  | SPI_CR1_SPE; //SPI enabled
        NVIC_SetPriority(DMA2_Stream5_IRQn,10);//Low priority for DMA
        NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    }
}

//
// class SPIRequest
//

void SPIRequest::IRQbeginTransaction()
{
    chipSelect.low();
    
    // DMA2, Chan 3, Stream 0: RX; Stream 3: TX;
    
    // Rx
    DMA2_Stream0->CR   = 0;
    DMA2_Stream0->PAR  = reinterpret_cast<unsigned int>(&SPI1->DR);
    DMA2_Stream0->M0AR = reinterpret_cast<unsigned int>(fromPeripheral.data());
    DMA2_Stream0->NDTR = fromPeripheral.size();
    DMA2_Stream0->CR   = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 // Channel 3
                       | DMA_SxCR_PL_1    //High priority because fifo disabled
                       | DMA_SxCR_MINC    //Increment memory pointer
                       | DMA_SxCR_TEIE    //Interrupt on transfer error
                       | DMA_SxCR_DMEIE   //Interrupt on direct mode error
                       | DMA_SxCR_EN;     //Start DMA
    
    // Tx
    DMA2_Stream5->CR   = 0;
    DMA2_Stream5->PAR  = reinterpret_cast<unsigned int>(&SPI1->DR);
    DMA2_Stream5->M0AR = reinterpret_cast<unsigned int>(toPeripheral.data());
    DMA2_Stream5->NDTR = toPeripheral.size();
    DMA2_Stream5->CR   = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0 // Channel 3
                       | DMA_SxCR_PL_1    //High priority because fifo disabled
                       | DMA_SxCR_MINC    //Increment memory pointer
                       | DMA_SxCR_DIR_0   //Memory to peripheral
                       | DMA_SxCR_TCIE    //Interrupt on transfer complete
                       | DMA_SxCR_TEIE    //Interrupt on transfer error 
                       | DMA_SxCR_DMEIE   //Interrupt on direct mode error
                       | DMA_SxCR_EN;     //Start DMA
    // FIXME: handle DMA_SxCR_TEIE
}
    
void SPIRequest::IRQendTransaction()
{
    //FIXME: ensure dead time between CS high and CS low!!!
    //And, is it really needed?
    chipSelect.high();
    
    //TX dma channel has no IRQ, so clear flag here
    DMA2->HIFCR=DMA_HIFCR_CTCIF5;
}
