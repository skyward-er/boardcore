#!/bin/bash

function check {
    B=$(eval "$3")
    A=$?
    echo -n "[LINTER] $2... "

    if [ "$1" == "OUT" ]; then
        if [ ${#B} -gt 0 ]; then
            A=$1
        fi;
    fi;
    if [ "$A" == "$1" ]; then
        echo -e "FAIL\n------------ OUTPUT ------------"
        echo "$B"
        echo "--------------------------------"
    else
        echo "OK"
    fi;
}

check 0 "Checking for files with lines longer than 80 characters" 'egrep -r ".{81}" src/' 
check OUT "Checking for files having \n\n\n" "grep -HPcrz '(\\r?\\n){4,}' src | egrep -v ':0$' | cut -d ':' -f 1"
check OUT "Checking for files not having the copyright" "grep -rL '* Permission is hereby granted, free of charge' src/"
check OUT "Checking for tabulations instead of spaces" "grep -Pr '\t' src"
check OUT "MMP wants his full name" "grep -rl 'Matteo Piazzolla' src/"
check OUT "Launching cppcheck" "cppcheck -q --suppress=unusedFunction --suppress=missingInclude --std=c++11 --enable=all src/ 2>&1"
check OUT "Checking for using namespace in header files" "grep -rl 'using namespace' src | egrep '.h(pp)?$'"

exit 0
