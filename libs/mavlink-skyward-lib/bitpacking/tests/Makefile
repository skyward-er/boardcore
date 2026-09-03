
TESTS = include/catch.hpp
REQUIREMENTS = test-bitpacker.cpp manual-test-bitpacker.cpp test-telemetry.cpp $(wildcard ../hermes/*.h) $(TESTS)

all: $(REQUIREMENTS)
	g++ -O2 -o test-bitpacker test-bitpacker.cpp
	g++ -O2 -o manual-test-bitpacker manual-test-bitpacker.cpp
	g++ -O2 -o test-telemetry test-telemetry.cpp -I../ -I../../

warning-all: $(REQUIREMENTS)
	g++ -O2 -Wall -Wextra -pedantic -o test-bitpacker test-bitpacker.cpp
	g++ -O2 -Wall -Wextra -pedantic -o manual-test-bitpacker manual-test-bitpacker.cpp
	g++ -O2 -Wall -Wextra -pedantic -o test-telemetry test-telemetry.cpp -I../ -I../../

test-bitpacker: test-bitpacker.cpp $(TESTS)
	g++ -O2 -o test-bitpacker test-bitpacker.cpp

manual-test-bitpacker: manual-test-bitpacker.cpp $(TESTS)
	g++ -O2 -o manual-test-bitpacker manual-test-bitpacker.cpp

test-telemetry: test-telemetry.cpp $(wildcard ../hermes/*.h) $(TESTS)
	g++ -O2 -o test-telemetry test-telemetry.cpp -I../ -I../../
	
clean:
	rm -rvf test-bitpacker manual-test-bitpacker test-telemetry
