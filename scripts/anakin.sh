#!/bin/bash

if [ $# -ne 1 ]; then
    echo "Usage: $0 <filename.bin>"
    exit -1
fi;

stm32flash -b 576000 -w $1 -v /dev/ttyUSB0
