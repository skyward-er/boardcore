#include "Rtu_Serial2.h"

using namespace std;
using namespace miosix;

typedef Gpio<GPIOA_BASE,1>  rts;
typedef Gpio<GPIOA_BASE,2>  tx;
typedef Gpio<GPIOA_BASE,3>  rx;

void serial2IRQHandler()
{
	if(USART2->SR | USART_SR_RXNE)
	{
		rxBuffer[rxBufIndex] = USART2->DR;
		rxBufIndex++;
	}
	
	if(USART2->SR | USART_SR_TXE)
	{
		if(txBufIndex < bytesToBeSent - 1)
		{
			USART2->DR = txBuffer[txBufIndex];
			txBufIndex++;
		}
	}
	
	if(USART2->SR | USART_SR_PE)
	{
		while(!(USART2->SR | USART_SR_TXE)) ; //wait for rx completion
		volatile uint8_t dummy = USART2->DR;  //dummy read to clear PE flag
	}
}

Rtu_Serial2::Rtu_Serial2() : timer(nullptr), txBufIndex(0), bytesToBeSent(0),
						rxBufIndex(0), newPacket(false), newDataIncoming(false)
{
	{
		FastInterruptDisableLock dLock;
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		RCC_SYNC();
		
		tx::mode(Mode::ALTERNATE);
		tx::alternateFunction(7);
		rx::mode(Mode::ALTERNATE);
		rx::alternateFunction(7);
		rts::mode(Mode::OUTPUT);
		rts::low();
	}
	
	NVIC_SetPriority(USART2_IRQn,15);//Lowest priority for serial
	NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
	
	// Enable parity, 9 bit frame length (9th bit is the parity one),
	// use even parity, generate interrupt in case of parity error, tx end,
	// new byte received
	USART2->CR1 |= USART_CR1_M
				| USART_CR1_PCE
				| USART_CR1_PEIE
				| USART_CR1_TXEIE
				| USART_CR1_RXNEIE
				| USART_CR1_TE		//enable tx
				| USART_CR1_RE;		//enable rx
	
  // CR2 register is left untouched since its default values are OK
  
  USART2->CR3 |= USART_CR3_ONEBIT;
}

Rtu_Serial2::~Rtu_Serial2()
{
	FastInterruptDisableLock dLock;
	RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
	RCC_SYNC();
}

void Rtu_Serial2::setBaud(uint32_t baud)
{
	uint32_t busFreq = SystemCoreClock;
	if(RCC->CFGR & RCC_CFGR_PPRE1_2)
	{
		busFreq/=1<<(((RCC->CFGR>>10) & 0x3)+1);
	}
	
	uint32_t quot=2*busFreq/baud; //2*freq for round to nearest
    USART2->BRR=quot/2 + (quot & 1); 
}

void Rtu_Serial2::sendTo(uint8_t address, std::unique_ptr< PDU >& pdu)
{
	txBuffer[0] = address;
	txBuffer[1] = pdu->funcCode();
	auto data = pdu->data();
	memcpy(&txBuffer[2],data.second,data.first);
	uint16_t crc = CRC16(data.second,data.first);
	
	// CRC is located 2 + dataLen bytes ahead from the beginning of the buffer.
	// This in array index terms, means from dataLen - 1
	uint8_t crcStart = data.first - 1;
	txBuffer[crcStart] = static_cast< uint8_t >((crc >> 8) & 0xFF);
	txBuffer[crcStart + 1] = static_cast< uint8_t >(crc & 0xFF);
	
	bytesToBeSent = data.first + 4;
}

uint16_t Rtu_Serial2::CRC16(uint8_t* data, size_t len)
{
	uint16_t crc = 0xFFFF;
    for(size_t i=0; i < len; i++) {
        
        crc ^= static_cast<uint16_t>(data[i]);
        for(int j=0; j<8; j++) {
            
            if(crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            }else {
                crc >>= 1;
            }
        }
    }    
    return crc;
}


