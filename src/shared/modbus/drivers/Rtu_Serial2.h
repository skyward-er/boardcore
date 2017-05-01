#ifndef RTU_SERIAL2_H
#define RTU_SERIAL2_H

#include "Common.h"
#include "../PDU.h"
#include <memory>
#include <cstring>

class Rtu_Serial2 : public Singleton< Rtu_Serial2 >
{
friend class Singleton< Rtu_Serial2 >;
	
public:
	~Rtu_Serial2();
	void setBaud(uint32_t baud);
	void assignTimer(Timer *tim);
	void sendTo(uint8_t address, std::unique_ptr<PDU>& pdu);
	void sendReply(std::unique_ptr<PDU>& pdu);
	bool newPacketReceived();
	uint8_t readPacketAddress();
	std::unique_ptr<PDU> getPacketPdu();

private:
	Rtu_Serial2();
	Rtu_Serial2(const Rtu_Serial2& other) = delete;
	Rtu_Serial2& operator=(const Rtu_Serial2& other) = delete;
	bool operator==(const Rtu_Serial2& other) = delete;
	
	friend void serial2IRQHandler();
	
	uint16_t CRC16(uint8_t *data, size_t len);
	
	Timer *timer;
	bool newPacket;
	bool newDataIncoming;	
	
	uint8_t rxBuffer[256];
	uint8_t rxBufIndex;
	
	uint8_t txBuffer[256];
	uint8_t txBufIndex;
	uint8_t bytesToBeSent;
};

#endif // RTU_SERIAL2_H
