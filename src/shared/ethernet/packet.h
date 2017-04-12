#ifndef PACKET_H
#define PACKET_H

#include <cstdint>
#include <cstdio>

//UDP packet header descriptor
typedef struct __attribute__((packed)) {
    uint32_t ipAddress;
    uint16_t port;
    size_t payloadSize;    
} packet_header_t;

#endif