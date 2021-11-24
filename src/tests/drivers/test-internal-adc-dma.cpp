/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <drivers/adc/InternalADC.h>
#include <miosix.h>

ADC_TypeDef& ADCx                = *ADC3;
DMA_Stream_TypeDef* DMAx_Streamx = DMA2_Stream1;

int main()
{

    // DMA INIT ---------------------------------------------------------------
    {
        // 1: Disable the stream

        // Disable stream
        DMAx_Streamx->CR &= ~DMA_SxCR_EN;

        // Wait for the stream to be disabled
        while (DMAx_Streamx->CR & DMA_SxCR_EN)
            ;

        // Reset DMA configuration
        DMAx_Streamx->CR = 0;

        // Ensures all the status bits are cleared by explicitly clearing them
        DMA2->LIFCR = 0x0F7D0F7D;  // Mask excluding reserved bits
        DMA2->HIFCR = 0x0F7D0F7D;

        // 2-4: The memory and peripheral addresses, also the number of data
        // items will be set by the driver

        // 5: Select the DMA channel
        DMAx_Streamx->CR |= DMA_SxCR_CHSEL_1;  // <- CHANGE THIS!

        // 6: The peripheral is not intended to be the flow controller
        // (it will function only as a trigger)
        DMAx_Streamx->CR &= ~DMA_SxCR_PFCTRL;

        // 7: Configure the stream priority to very high
        DMAx_Streamx->CR |= DMA_SxCR_PL;

        // 8: Configure fifo usage (disable => direct mode)
        DMAx_Streamx->FCR &= ~DMA_SxFCR_DMDIS;

        // 9: Other configuration

        // Data transfer peripheral-to-memory
        DMAx_Streamx->CR &= ~DMA_SxCR_DIR;

        // Address increment mode
        DMAx_Streamx->CR &= ~DMA_SxCR_PINC;  // Fixed
        DMAx_Streamx->CR |= DMA_SxCR_MINC;   // Incremented

        // Single or burst transaction (single)
        DMAx_Streamx->CR &= ~DMA_SxCR_MBURST;

        // Data width (16 bit)
        DMAx_Streamx->CR |= DMA_SxCR_PSIZE_0;
        DMAx_Streamx->CR |= DMA_SxCR_MSIZE_0;

        // Circular mode (on)

        DMAx_Streamx->CR |= DMA_SxCR_CIRC;

        // Double buffer mode (off)
        DMAx_Streamx->CR &= ~DMA_SxCR_DBM;

        // Interrupt not used

        // 10: Activate the stream after peripheral configuration (see warning)
    }

    // Set pins PA0 PA1 PA2 PA3 as analog input
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER = 0xFF;

    // Enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;  // <-- CHANGE THIS!

    // Set the clock divider for the analog circuitry (/8)
    ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;
    // In this case I've set the maximum value, check the datasheet for the
    // maximum frequency the analog circuitry supports and compare it with the
    // parent clock

    InternalADC adc(ADCx, 3.0, DMAx_Streamx);
    adc.enableChannel(InternalADC::CH0);
    adc.enableChannel(InternalADC::CH3);
    adc.init();

    printf("Configuration completed\n");

    while (1)
    {
        adc.sample();

        printf("%8lld:%.2f\t%8lld:%.2f\n",
               adc.getVoltage(InternalADC::CH0).adc_timestamp,
               adc.getVoltage(InternalADC::CH0).voltage,
               adc.getVoltage(InternalADC::CH0).adc_timestamp,
               adc.getVoltage(InternalADC::CH3).voltage);

        miosix::delayMs(1000);
    }
}