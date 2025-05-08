BOARDCORE := ../../

all:
	g++ -std=c++20 -O2 -o logdecoder logdecoder.cpp \
					-DCOMPILE_FOR_X86 \
					-DCOMPILE_FOR_HOST \
					-DFMT_HEADER_ONLY \
					-I$(BOARDCORE)libs/miosix-host \
	 				-I$(BOARDCORE)libs/mavlink-skyward-lib \
	 				-I$(BOARDCORE)libs/eigen \
	 				-I$(BOARDCORE)libs/socrate/include \
	 				-I$(BOARDCORE)libs/fmt/include \
	 				-I$(BOARDCORE)src/shared 
clean:
	rm logdecoder
