#!/bin/bash

if [ $# -ne 1 ]; then
    echo "Usage: $0 <filename.bin>"
    exit -1
fi;

arm-miosix-eabi-gdb -ex 'target remote :3333' -ex 'monitor reset halt'  -ex 'monitor reset halt'  -ex 'monitor reset halt' -ex "monitor flash write_image erase $1 0x08000000" -ex 'continue&' -ex 'set confirm off' -ex 'quit'
