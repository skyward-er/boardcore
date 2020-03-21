#!/bin/bash

#
# Usage: grepcheck <FOLDER_TO_CHECK> <STRING_TO_FIND> <EGREP_ARGS>
#
function grepcheck {

    FILES=$(grep -rl "$2" $1 | egrep "$3")

    for f in $FILES
    do
        # Print name of the file in red
        echo -ne '\033\x5b\x33\x33\x6d'

        echo $f
        echo -ne '\033\x5b\x30\x6d'

        if [[ $verbose -eq 1 ]]; then
            grep --color=always -r "$2" $f -C3
            echo ''
        fi
    done

    echo ''

    # If VERBOSE is defined then also print the context in the file
}

# Horrifying way of checking if -v or --verbose is enabled
verbose=0

case "$1" in
-v|--verbose)
    verbose=1
    shift ;;
esac

case "$2" in
-v|--verbose)
    verbose=1
esac

# Execute checks
echo '--------------------------- CPPCHECK'
echo '[Linter] Executing cppcheck'
echo ''
cppcheck  -U "RCC_APB1ENR_CAN2EN" --template=gcc -q --std=c++11 --enable=all \
            --suppress=unusedFunction --suppress=missingInclude --suppress=noExplicitConstructor \
            "$1" 2>&1 | awk '
                function color(c,s) {
                        printf("\033[%dm%s\033[0m\n",30+c,s)
                }
                /error/ {color(1,$0);next}
                /warning/ {color(3,$0);next}
                /style/ {color(2,$0);next}
                /performance/ {color(6,$0);next}
                /information/ {color(4,$0);next}
                /portability/ {color(5,$0);next}
                {print}
                '
echo ''

echo '--------------------------- USING NAMESPACE'
echo '[Linter] Checking for using namespace in .h(pp) files'

grepcheck $1 'using namespace' '.h(pp)?$'

echo '--------------------------- GREP PRINTF'
echo '[Linter] Checking for printf instead of TRACE'

grepcheck $1 "printf" '.*'

echo '--------------------------- GREP ASSERT'
echo '[Linter] Checking if there are asserts in the code'

grepcheck $1 "assert" '.*'

echo '--------------------------- GREP THROW'
echo '[Linter] Checking if there are exceptions in the code'

grepcheck $1 "throw" '~*catch.hpp'

#exit 0
