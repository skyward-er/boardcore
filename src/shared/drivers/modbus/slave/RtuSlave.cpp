#include "RtuSlave.h"

using namespace std;
using namespace miosix;

typedef Gpio<GPIOA_BASE,2>  u2tx;
typedef Gpio<GPIOA_BASE,3>  u2rx;
typedef Gpio<GPIOA_BASE,1>  u2rts;

typedef Gpio<GPIOB_BASE,10> u3tx;
typedef Gpio<GPIOB_BASE,11> u3rx;
typedef Gpio<GPIOB_BASE,14> u3rts;

void __attribute__((naked)) TIM1_BRK_TIM9_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z13TimIRQHandlerv");
    restoreContext();
}

void __attribute__((weak)) USART2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z14Ser2IRQHandlerv");
    restoreContext();
}

void __attribute__((weak)) USART3_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z14Ser3IRQHandlerv");
    restoreContext();
}

void __attribute__((used)) TimIRQHandler()
{
    
    auto inst = Singleton< RtuSlave >::getInstance();
    inst->IRQTimerInterrupt();        
}

void __attribute__((used)) Ser2IRQHandler()
{
    
    auto inst = Singleton< RtuSlave >::getInstance();
    inst->IRQSerial2Interrupt();
}

void __attribute__((used)) Ser3IRQHandler()
{
    
    auto inst = Singleton< RtuSlave >::getInstance();
    inst->IRQSerial3Interrupt();
}

RtuSlave::RtuSlave() : timer_k(1.0f)
{
    memset(serial2_data.rxBuffer,0,256);
    serial2_data.rxWriteIndx = 0;
    serial2_data.lastEvent = 0;
    serial2_data.rxInProgress = false;
    
    memset(serial2_data.txBuffer,0,256);
    serial2_data.txReadIndx = 0;
    serial2_data.txSize = 0;
    
    memset(serial3_data.rxBuffer,0,256);
    serial3_data.rxWriteIndx = 0;
    serial3_data.lastEvent = 0;
    serial3_data.rxInProgress = false;
    
    memset(serial3_data.txBuffer,0,256);
    serial3_data.txReadIndx = 0;
    serial3_data.txSize = 0;
    
    {
        FastInterruptDisableLock dLock;
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
        RCC_SYNC();
        
        u2tx::mode(Mode::ALTERNATE);
        u2rx::mode(Mode::ALTERNATE);
        u2tx::alternateFunction(7);
        u2rx::alternateFunction(7);
        u2rts::mode(Mode::OUTPUT);
        u2rts::low();
        
        u3tx::mode(Mode::ALTERNATE);
        u3rx::mode(Mode::ALTERNATE);
        u3tx::alternateFunction(7);
        u3rx::alternateFunction(7);
        u3rts::mode(Mode::OUTPUT);
        u3rts::low();        
    }
    
    /*** USART 2 configuration ***/
    // Enable parity, 9 bit frame length (9th bit is the parity one),
    // use even parity, generate interrupt in case of parity error, tx end,
    // new byte received
    USART2->CR1 |= USART_CR1_M
                | USART_CR1_PCE
                | USART_CR1_PEIE
                | USART_CR1_TXEIE
                | USART_CR1_RXNEIE
                | USART_CR1_TE      //enable tx
                | USART_CR1_RE;     //enable rx
    
    // CR2 register is left untouched since its default values are OK
  
    USART2->CR3 |= USART_CR3_ONEBIT;

    NVIC_SetPriority(USART2_IRQn,15);//Lowest priority for serial
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);

    /*** USART 3 configuration, same as USART 2 ***/
    USART3->CR1 |= USART_CR1_M
                | USART_CR1_PCE
                | USART_CR1_PEIE
                | USART_CR1_TXEIE
                | USART_CR1_RXNEIE
                | USART_CR1_TE      //enable tx
                | USART_CR1_RE;     //enable rx

    USART3->CR3 |= USART_CR3_ONEBIT;

    NVIC_SetPriority(USART3_IRQn,15);//Lowest priority for serial
    NVIC_ClearPendingIRQ(USART3_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);    
}

RtuSlave::~RtuSlave()
{    
    FastInterruptDisableLock dLock;
    
    TIM9->CR1 &= ~TIM_CR1_CEN;
    
    RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
    RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN;    
    RCC_SYNC();        
}

void RtuSlave::setBaud(uint32_t baud)
{
    uint32_t busFreq = SystemCoreClock;
    if(RCC->CFGR & RCC_CFGR_PPRE1_2)
    {
        busFreq/=1<<(((RCC->CFGR>>10) & 0x3)+1);
    }
    
    uint32_t quot=2*busFreq/baud; //2*freq for round to nearest
    USART2->BRR=quot/2 + (quot & 1);
    USART3->BRR=quot/2 + (quot & 1);
    
    timerInit(baud);
}

bool RtuSlave::newDataReceived(uint8_t interface)
{
    if(interface == 1)
    {
        if(!(serial2_data.rxInProgress) && (serial2_data.rxWriteIndx > 0))
        {
            return true;
        }
    }
    
    if(interface == 2)
    {
        if(!(serial3_data.rxInProgress) && (serial3_data.rxWriteIndx > 0))
        {
            return true;
        }
    }
    
    return false;
}

pair< uint8_t, unique_ptr< PDU > > RtuSlave::readData(uint8_t fromInterface)
{
    //uint8_t address =  TODO incomplete?
}

void RtuSlave::timerInit()
{
    TIM9->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;
    TIM9->ARR = 0xFFFF;
    TIM9->CCMR1 = 0;
    TIM9->CCMR2 = 0;
    TIM9->CNT = 0;    
    
    NVIC_SetPriority(TIM9_IRQn,15);//Lowest priority for serial
    NVIC_ClearPendingIRQ(TIM9_IRQn);
    NVIC_EnableIRQ(TIM9_IRQn);
    
    uint32_t busFreq = SystemCoreClock;
    if(RCC->CFGR & RCC_CFGR_PPRE2_2)
    {
        busFreq/=1<<(((RCC->CFGR>>13) & 0x3)+1);
    }
    
    /* Here we calculate the prescaler value based on baud rate value.
     * From the modbus RTU specification we have that, if the baud is greater 
     * than 19200, the t1.5 and t3.5 are fixed as the baud is 19200.
     * The timer is set up to accept values multiple of t1.5. Its exact value is
     * given by the formula (1.5 * 11) / baud, since a modbus RTU character is 
     * 11 bit long. Here we use an aproximate one (17/baud instead of 16.5/baud)
     * because we do all the calculations with ints.
     * The prescaler value is given by busFreq * t1.5, that is 
     * (busFreq * 17) / baud. If this value falls above 65536, the maximum 
     * acceptable for the prescaler, the latter is set to 0xFFFF and a suitable
     * value for the correction factor k is computed.
     */
    uint32_t prescaler = (busFreq / min(baud,19200)) * 17;
    if(prescaler > 65536)
    {
        TIM9->PSC = 0xFFFF;
        timer_k = (static_cast< float >(busFreq) * 17.0f);
        timer_k /= (65536.0f * static_cast< float >(baud));
    }else
    {
        TIM9->PSC = prescaler - 1;
        timer_k = 1.0f;
    }
    
    TIM9->EGR |= TIM_EGR_UG;    //generate update event to load values
    TIM9->CR1 |= TIM_CR1_CEN;
}

void RtuSlave::setNewTimeout(uint8_t channel, uint8_t ticks)
{
    uint16_t incr = static_cast< uint16_t >((ticks * timer_k) + 0.5f);
    switch(channel)
    {
        case 1:
            TIM9->CCMR1 += incr;
            break;
            
        case 2:
            TIM9->CCMR2 += incr;
            break;
        
        default:
            break;
    }
}

void RtuSlave::IRQTimerInterrupt()
{
    if(TIM9->SR | TIM_SR_CC1IF)
    {
        TIM9->SR &= ~TIM_SR_CC1IF;
        serial2_data.rxInProgress = false;
    }
    
    if(TIM9->SR | TIM_SR_CC2IF)
    {
        TIM9->SR &= ~TIM_SR_CC2IF;
        serial3_data.rxInProgress = false;
    }
}

void RtuSlave::IRQSerial2Interrupt()
{
    if(USART2->SR & USART_SR_RXNE)
    {
        if(!serial2_data.rxInProgress && serial2_data.rxWriteIndx == 0)
        {
            serial2_data.rxInProgress = true;
        }
        
        if(serial2_data.rxInProgress)
        {
            serial2_data.rxBuffer[serial2_data.rxWriteIndx] = USART2->DR;
            serial2_data.rxWriteIndx++;
            //new character has to be received in 1.5*character_time instants
            setNewTimeout(1,1);
        }
    }
    
    if(USART2->SR & USART_SR_TXE)
    {
        if(serial2_data.txReadIndx < serial2_data.txSize)
        {
            USART->DR = serial2_data.txBuffer[serial2_data.txReadIndx];
            serial2_data.txReadIndx++;
        }
    }
}
