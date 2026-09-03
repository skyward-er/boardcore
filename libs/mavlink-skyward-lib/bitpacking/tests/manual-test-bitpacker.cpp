
#include "../BitPacker.h"
#include <stdint.h>
#include <stdio.h>

int main() {

    uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
    BitPacker packer(array);
    
    unsigned int pos;
    unsigned int bits;
    uint64_t value;
    uint64_t result;

    printf("%ld\n", sizeof(uint64_t));

    while(1) {
        printf("\nposition: ");
        scanf("%u", &pos);
        printf("number of bits: ");
        scanf("%u", &bits);
        printf("value: ");
        scanf("%lu", &value);
        packer.pack(pos, bits, value);
        packer.printPacket();
        packer.unpack(pos, bits, &result);
        printf("result: %ld\n", result);
    }
}