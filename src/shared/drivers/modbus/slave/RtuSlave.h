#ifndef RTUSLAVE_H
#define RTUSLAVE_H

#include <memory>
#include "../../Common.h"
#include "../PDU.h"
#include "Timer.h"

class RtuSlave : public Singleton< RtuSlave >
{
    
friend class Singleton< RtuSlave >;
public:
    ~RtuSlave();
    
    void setBaud(uint32_t baud) throw();
            
    void sendReply(std::unique_ptr<PDU>& data, uint8_t fromInterface);
    
    bool newDataReceived(uint8_t interface);
    
    std::pair<uint8_t, std::unique_ptr<PDU>> readData(uint8_t fromInterface);
    
    void IRQSerial2Interrupt();
    void IRQSerial3Interrupt();
    void IRQTimerInterrupt();
    
private:
    RtuSlave();
    
    void timerInit();
    void setNewTimeout(uint8_t channel, uint8_t ticks);
    
    typedef struct
    {
        uint8_t rxBuffer[256];
        uint8_t rxWriteIndx;
        uint16_t lastEvent;
        bool rxInProgress;
    
        uint8_t txBuffer[256];
        uint8_t txReadIndx;
        uint8_t txSize;
    
    }interf_data;
    
    interf_data serial2_data;
    interf_data serial3_data;
    
    float timer_k;
    
    RtuSlave(const RtuSlave& other) = delete;
    RtuSlave& operator=(const RtuSlave& other) = delete;
    bool operator==(const RtuSlave& other) = delete;       
};

#endif // RTUSLAVE_H
