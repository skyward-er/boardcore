#ifndef PACKETBUFFER_H
#define PACKETBUFFER_H

#include <Common.h>
#include "packet.h"

/**
 * Quick and dirty implementation of a ring buffer. All the operations are
 * guaranteed to be atomic through the use of a mutex
 */
class PacketBuffer
{
public:
    
    /**
     *  @param storageSize: size of internal buffer in bytes
     *  internal buffer is made of uint8_t
     */
    PacketBuffer(size_t storageSize);
    
    ~PacketBuffer();
    
    /**
     * Check if internal buffer's allocation succeeded. Call this function
     * immediately after constructor and, if it returns false, call the
     * destructor.
     * @return true on success, false on failure.
     */
    bool isValid() { return valid; }
    
    /**
     * Enqueues a packet at list's tail. If it cannot be enqueued is dropped
     * @param packet packet to be enqueued descriptor sctructure
     * @return true if packet can be enqueued, false otherwise
     */
    bool push(packet_header_t& header, const uint8_t *payload);
    

    packet_header_t getHeader();
    
    void getData(uint8_t *data);
    
    /**
     * Removes the packet placed at list's head by avancing it of the
     * packet's size
     */   
    void popFront();

    /**
     * @return true if buffer is empty
     */
    bool empty();
    
private:
    size_t storageSize;
    size_t usedSize;
    uint32_t writeIndex;
    uint32_t readIndex;
    bool valid;
    uint8_t *buffer;
    
    miosix::FastMutex mutex;
    
    //Copy constructor and copy assignment are not allowed
    PacketBuffer(PacketBuffer& other);
    PacketBuffer& operator=(const PacketBuffer& other);
};

#endif // PACKETBUFFER_H
