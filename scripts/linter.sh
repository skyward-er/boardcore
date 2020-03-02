#!/bin/bash

echo '[Linter] Executing cppcheck'
cppcheck  --template=gcc -q --std=c++11 --enable=all \
            --suppress=unusedFunction --suppress=missingInclude --suppress=noExplicitConstructor \
            "$@" 2>&1 | awk '
                function color(c,s) {
                        printf("\033[%dm%s\033[0m\n",30+c,s)
                }
                /warning/ {color(1,$0);next}
                /style/ {color(2,$0);next}
                /performance/ {color(3,$0);next}
                /information/ {color(4,$0);next}
                /portability/ {color(5,$0);next}
                {print}
                '

#exit 0
