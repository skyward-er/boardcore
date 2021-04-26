BASE := ../../../../../

all:
	g++ -std=c++17 -O2 -o logdecoder logdecoder.cpp \
					-DCOMPILE_FOR_X86 \
					$(BASE)libs/tscpp/stream.cpp \
	 				-I$(BASE)src/shared \
	 				-I$(BASE)src/tests \
					-I$(BASE)libs \
					-I$(BASE)libs/miosix-kernel/miosix

clean:
	rm logdecoder
