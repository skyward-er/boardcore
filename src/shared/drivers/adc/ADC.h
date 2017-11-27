/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Michele Piazzolla
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

#include "Common.h"
#include "Sensor.h"

template <unsigned N, unsigned CHANNEL, class GpioADC>
class SensorADC
{
public:
    SensorADC(uint8_t stime)
    {
        ADCx         = getADC(N);
        samplingTime = stime;
    }

    bool init()
    {
        GpioADC::mode(miosix::Mode::INPUT_ANALOG);
        enableADC(ADCx);

        ADCx->SQR1 = 0x0;  // One conversion
        ADCx->SQR3 = CHANNEL;

        ADCx->CR2 = ADC_CR2_SWSTART | ADC_CR2_CONT | ADC_CR2_ADON;
        // CCR pre scaler
        ADC->CCR = 0x3 << 16;

        // ADCx->SMPR1= (0x7<<6)<<3;

        // set channel sampling time
        if (CHANNEL < 10)
        {
            ADCx->SMPR2 = (samplingTime << (3 * (CHANNEL)));
        }
        if (CHANNEL >= 10)
        {
            ADCx->SMPR1 = (samplingTime << (3 * (CHANNEL - 10)));
        }

        return true;
    }

    static inline void enableADC(ADC_TypeDef* adc)
    {
        miosix::FastInterruptDisableLock dLock;

        if (adc == ADC1)
        {
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
            printf("ADC1\n");
        }
        else if (adc == ADC2)
        {
            RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
            printf("ADC2\n");
        }
        else if (adc == ADC3)
        {
            RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
            printf("ADC3\n");
        }

        RCC_SYNC();
    }

    bool selfTest() { return false; }

    bool updateParams()
    {
        ADCx->CR2 = ADC_CR2_SWSTART | ADC_CR2_CONT | ADC_CR2_ADON;

        miosix::Thread::sleep(10);
        ADCx->CR2 |= ADC_CR2_ADON;  // Setting ADC ON twice starts a conversion
        while ((ADCx->SR & ADC_SR_EOC) == 0)
            ;
        last_value = ADCx->DR;
        return true;
    }

    constexpr ADC_TypeDef* getADC(unsigned n)
    {
        return n == 1 ? ADC1 : n == 2 ? ADC2 : ADC3;
    }

    uint16_t getValue() { return last_value; }

    enum channelSampling
    {
        CYCLES_3   = 0x0,
        CYCLES_15  = 0x1,
        CYCLES_28  = 0x2,
        CYCLES_56  = 0x3,
        CYCLES_84  = 0x4,
        CYCLES_112 = 0x5,
        CYCLES_144 = 0x6,
        CYCLES_480 = 0x7
    };

private:
    uint16_t last_value;
    uint8_t samplingTime;
    ADC_TypeDef* ADCx;
};
