#include "PacketBuffer.h"

using namespace std;
using namespace miosix;

PacketBuffer::PacketBuffer(size_t storageSize) : storageSize(storageSize), 
                                    writeIndex(0), readIndex(0), valid(true)  {
    #ifndef __NO_EXCEPTIONS
    try {
        buffer = new uint8_t[storageSize];
    }catch(std::bad_alloc& exc)
    {
        std::cout << "bad alloc! " << exc.what() << endl;
        valid = false;
    }
    #else
    buffer = new (std::nothrow) uint8_t[storageSize];
    if(buffer == nullptr) {
        valid = false;
    }
    #endif
}

PacketBuffer::~PacketBuffer() {
    
    if(valid) {
        delete[] buffer;
    }
}

bool PacketBuffer::push(packet_header_t& header, const uint8_t* payload) {
    
    if(!valid) {
        return false;
    }
    
    size_t usedSpace = sizeof(packet_header_t) + header.payloadSize;

    // We had to split the check into two sections because of this corner case:
    // readIndex and writeIndex are initially set to zero and we attempt to 
    // write something to the buffer. In this case the check 
    // (writeIndex + usedSpace) % storageSize >= readIndex will fail because 
    // readIndex is zero
    
    bool overflows = (writeIndex + usedSpace) >= storageSize;
    bool overwrites = (writeIndex + usedSpace) % storageSize >= readIndex;

    if(readIndex == 0 && overflows) {
        return false;
    }
    
    if(readIndex != 0 && overwrites) {
        return false;
    }
    
    // to store the packet into the ring buffer first copy the header
    // and then copy the payload
    {
        Lock< FastMutex > l(mutex);
        uint8_t *head = reinterpret_cast< uint8_t* >(&header);
        
        for(unsigned int i = 0; i < sizeof(packet_header_t); i++) {
            buffer[(writeIndex + i) % storageSize] = head[i];
        }
        
        writeIndex = (writeIndex + sizeof(packet_header_t)) % storageSize;
        
        for(unsigned int i = 0; i < header.payloadSize; i++) {
            buffer[(writeIndex + i) % storageSize] = payload[i];
        }
        
        writeIndex = (writeIndex + header.payloadSize) % storageSize;
    }
    
    return true;
}

packet_header_t PacketBuffer::getHeader() {

    packet_header_t header;
    uint8_t *dest = reinterpret_cast< uint8_t* >(&header);
    memset(dest,0x00,sizeof(packet_header_t));
    
    if(valid) {
        Lock< FastMutex > l(mutex);
        for(unsigned int i=0; i<sizeof(packet_header_t); i++) {
            dest[i] = buffer[(readIndex + i) % storageSize];
        }
    }
    return header;
}

void PacketBuffer::getData(uint8_t* data) {
    
    if(!valid) {
        return;
    }
    
    /* Packet size is stored in header, so first get the payloadSize.
     * The data is stored sizeof(packet_header_t) bytes next the read pointer,
     * since a packet is stored as header followed by payload, so we have to
     * add sizeof(packet_header_t) to the read pointer to reach the first data
     * byte
     */
    
    packet_header_t frontHead = getHeader();
    
    {
        Lock< FastMutex > l(mutex);
        int start = readIndex + sizeof(packet_header_t);
        
        for(unsigned int i=0; i<frontHead.payloadSize; i++) {
            data[i] = buffer[(start + i) % storageSize];
        }
    }
}

void PacketBuffer::popFront() {
    
    if((!valid) || empty()) { //this means that list is empty
        return;
    }
    
    /* Popping a packet out of list simply means move the readIndex forward
     * of the size of the packet at list's head.
     * The size of a packet is made of two parts: a fixed one which is
     * constituded by the size of IP address, port and packet length fields
     * and a variable one that is the size of the payload. The latter is stored
     * into the len field of the packet.
     * What we do here is gather the fixed and variable sizes and sum them
     * together to obtain the readIndex's needed increment       
     */
    
    packet_header_t head = getHeader();
    size_t packetSize = sizeof(packet_header_t) + head.payloadSize;

    {
        Lock< FastMutex > l(mutex);
        readIndex = (readIndex + packetSize) % storageSize;
    }
}

bool PacketBuffer::empty() {
    
    Lock< FastMutex > l(mutex);
    return readIndex == writeIndex;
}
