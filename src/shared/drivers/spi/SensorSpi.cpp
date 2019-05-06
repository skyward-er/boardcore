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

#include "SensorSpi.h"
#include <kernel/scheduler/scheduler.h>
#include <interfaces-impl/hwmapping.h>

using namespace std;
using namespace miosix;

static Thread* waiting                   = nullptr;
static vector<SPIRequest>* requestVector = nullptr;
static size_t requestIndex               = 0;
static bool error                        = false;
static uint32_t fifoFaultCtr             = 0;

/**
 * DMA RX end of transfer
 */
void __attribute__((naked)) DMA2_Stream0_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20SPI1rxDmaHandlerImplv");
    restoreContext();
}

/**
 * DMA RX end of transfer actual implementation
 */
void __attribute__((used)) SPI1rxDmaHandlerImpl()
{
    if (DMA2->LISR & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0))
    {
        error = true;
        fifoFaultCtr++;
    }

    // FEIF3 is not checked because is ALWAYS 1 both if FIFO is enabled
    // or disabled. Tests show that all data are transferred correctly!
    if (DMA2->LISR & (DMA_LISR_TEIF3 | DMA_LISR_DMEIF3))
    {
        error = true;
        fifoFaultCtr++;
    }

    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 |
                  DMA_LIFCR_CFEIF0;

    DMA2->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CFEIF3;

    if (requestVector == nullptr)
        return;
    (*requestVector)[requestIndex].IRQendTransaction();

    if (++requestIndex >= requestVector->size())
    {
        waiting->IRQwakeup();
        if (waiting->IRQgetPriority() >
            Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            Scheduler::IRQfindNextThread();
        }

        waiting = 0;
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
    if (requests.empty())
        return false;
    for (auto& req : requests)
        if (req.empty())
            return false;

    pthread_mutex_lock(&mutex);
    error = false;
    enableDMA();

    waiting       = Thread::getCurrentThread();
    requestIndex  = 0;
    requestVector = &requests;

    {
        FastInterruptDisableLock dLock;

        (*requestVector)[requestIndex].IRQbeginTransaction();

        while (waiting != 0)
        {
            waiting->IRQwait();
            {
                FastInterruptEnableLock eLock(dLock);
                Thread::yield();
            }
        }
    }

    disableDMA();
    requestVector = nullptr;
    bool result   = !error;
    pthread_mutex_unlock(&mutex);
    return result;
}

SPIDriver::SPIDriver()
{
    pthread_mutex_init(&mutex, 0);
    {
        // Interrupts are disabled to prevent bugs if more than one threads does
        // a read-modify-write to RCC  or GPIO->MODER registers at the same time
        FastInterruptDisableLock dLock;
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

        /*
         * Table of the maximum speed of each sensor, used to set SPI speed
         * LSM9DS0   IMU           SPI <10MHz
         * MPU9250   IMU           SPI < 1MHz
         * MAX21105  IMU           SPI <10MHz
         * FXAS21002 gyro          SPI < 2MHz
         * MS5803    baro          SPI <20MHz
         * LPS331    baro          SPI FIXME: unknown!
         * MAX31856  thermocouple  SPI < 5MHz
         */

        disableDMA();
        NVIC_SetPriority(DMA2_Stream0_IRQn, 10);  // Low priority for DMA
        NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    }
}

uint32_t SPIDriver::getFIFOFaultCtr() const { return fifoFaultCtr; }

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

    DMA2_Stream0->FCR = DMA_SxFCR_FEIE     // Enable interrupt on FIFO error
                        | DMA_SxFCR_DMDIS  // Enable FIFO
                        | DMA_SxFCR_FTH_0  // FTH = 11 -> Full FIFO
                        | DMA_SxFCR_FTH_1;

    DMA2_Stream0->CR = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0  // Channel 3
                       | DMA_SxCR_PL_1   // High priority because fifo disabled
                       | DMA_SxCR_MINC   // Increment memory pointer
                       | DMA_SxCR_TCIE   // Interrupt on transfer complete
                       | DMA_SxCR_TEIE   // Interrupt on transfer error
                       | DMA_SxCR_DMEIE  // Interrupt on direct mode error
                       | DMA_SxCR_EN;    // Start DMA

    // Tx
    DMA2_Stream3->CR   = 0;
    DMA2_Stream3->PAR  = reinterpret_cast<unsigned int>(&SPI1->DR);
    DMA2_Stream3->M0AR = reinterpret_cast<unsigned int>(toPeripheral.data());
    DMA2_Stream3->NDTR = toPeripheral.size();

    DMA2_Stream3->FCR = DMA_SxFCR_FEIE     // Enable interrupt on FIFO error
                        | DMA_SxFCR_DMDIS  // Enable FIFO
                        | DMA_SxFCR_FTH_0  // FTH = 11 -> Full FIFO
                        | DMA_SxFCR_FTH_1;

    DMA2_Stream3->CR = DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0  // Channel 3
                       | DMA_SxCR_PL_1   // High priority because fifo disabled
                       | DMA_SxCR_MINC   // Increment memory pointer
                       | DMA_SxCR_DIR_0  // Memory to peripheral
                       | DMA_SxCR_TEIE   // Interrupt on transfer error
                       | DMA_SxCR_DMEIE  // Interrupt on direct mode error
                       | DMA_SxCR_EN;    // Start DMA
}

void SPIRequest::IRQendTransaction()
{
    chipSelect.high();
    /*
     * NOTE: this code has no explicit delays and relies on the code execution
     * timings to add the requred minimum delays between data CS toggling.
     * When running at 180MHz with data in external RAM, the delay are as follow
     * From last clock pulse till CS high : ~1.5us
     * From CS high to CS low             : ~2  us
     * From CS low to first clock pulse   : ~3.5us
     */
}
