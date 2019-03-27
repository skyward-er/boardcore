
all:
	@echo "Nothing to do (header only library). Run make test to run testsuite."

test:
	g++ -Wall -std=c++11 -fsanitize=address -O0 -g -o testsuite testsuite.cpp; ./testsuite; rm testsuite
