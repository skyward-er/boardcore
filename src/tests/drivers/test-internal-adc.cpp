#include <drivers/adc/InternalADC/InternalADC.h>
#include <miosix.h>
#include "TimestampTimer.h"

ADC_TypeDef& ADCx = *ADC3;

int main()
{
    // Set pins PA0 PA1 PA2 PA3 as analog input
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER = 0xFF;

    // Enable ADC3 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;  // <- CHANGE THIS!

    // Set the clock divider for the analog circuitry (/8)
    ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;

    // In this case I've set the maximum value, check the datasheet for the
    // maximum frequency the analog circuitry supports and compare it with the
    // parent clock

    TimestampTimer::enableTimestampTimer();

    InternalADC adc(ADCx, 3.0);
    adc.enableChannel(InternalADC::CH0);
    adc.enableChannel(InternalADC::CH2);
    adc.init();

    printf("Configuration completed\n");

    while (1)
    {
        adc.sample();

        printf("%f\t%f\n", adc.getVoltage(InternalADC::CH0).voltage,
               adc.getVoltage(InternalADC::CH2).voltage);

        miosix::delayMs(1000);
    }
}