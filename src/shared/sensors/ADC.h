using namespace miosix;

template<unsigned N,unsigned CHANNEL, class GpioADC>
class SensorADC : public Sensor {
public:
    SensorADC(){
        ADCx = getADC(N); 
    }

    bool init() {
		
		GpioADC::mode(Mode::INPUT_ANALOG);
		enableADC(ADCx);

        ADCx->SQR1 = 0x0; //One conversion
        ADCx->SQR3 = CHANNEL; // Channel 9
        
        ADCx->CR2 = ADC_CR2_SWSTART
                  | ADC_CR2_CONT
                  | ADC_CR2_ADON;
        //CCR pre scaler 
        ADC->CCR = 0x3<<16;
        
        ADCx->SMPR1= (0x7<<6)<<3;
		printf("DIOCANE\n");

        return true;
    }

    static inline void enableADC(ADC_TypeDef* adc) {
		{
		  FastInterruptDisableLock dLock;
			if(adc == ADC1){
				RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
				printf("ADC1\n");
			}
			else if(adc == ADC2){
				RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
				printf("ADC2\n");

			}
			else if(adc == ADC3){
				RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
				printf("ADC3\n");
			}
		}	

    }
    

    bool selfTest() {
    return false;
       
    }

    bool updateParams() {


    ADCx->CR2 = ADC_CR2_SWSTART
                  | ADC_CR2_CONT
                  | ADC_CR2_ADON;
                  
    Thread::sleep(10); //Wait for tempsensor to stabilize
    ADCx->CR2|=ADC_CR2_ADON; //Setting ADC ON twice starts a conversion
    while((ADCx->SR & ADC_SR_EOC)==0);
        last_value = ADCx->DR;
	return true;
    }


    
    
    constexpr ADC_TypeDef* getADC(unsigned n) {
        return  n==1 ? ADC1 :
                n==2 ? ADC2 : ADC3;
    }

    
    uint16_t getValue(){
        return last_value;
    }


private:

    uint16_t last_value;
    ADC_TypeDef* ADCx;


};
