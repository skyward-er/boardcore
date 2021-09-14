all: libfmt.a

libfmt.a: format.o
	arm-miosix-eabi-ar rvs libfmt.a format.o

format.o: src/format.cc
	arm-miosix-eabi-g++ -s -c -o format.o src/format.cc -O2 \
		-D_GLIBCXX_USE_WCHAR_T \
		-DFMT_UNICODE=0 -DFMT_STATIC_THOUSANDS_SEPARATOR=0\
		-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Iinclude
clean:
	rm -f format.o libfmt.a