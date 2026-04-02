MAKEFLAGS += --no-print-directory

BOARDCORE := ../../

CXX := g++
CXXFLAGS := -std=c++20 -O2 -s \
            -DCOMPILE_FOR_X86 \
            -DCOMPILE_FOR_HOST \
            -DFMT_HEADER_ONLY \
            -I$(BOARDCORE)libs/miosix-host \
            -I$(BOARDCORE)libs/socrate/include \
            -I$(BOARDCORE)libs/fmt/include \
            -I$(BOARDCORE)src/shared

# Default to dynamic linking
STATIC ?= 0
STATIC_STR :=
ifeq ($(STATIC), 1)
    LDFLAGS += -static
	STATIC_STR := " \(statically linked\)"
endif

.PHONY: all static clean .FORCE

.FORCE:

all: logdecoder

static:
	@$(MAKE) all STATIC=1

logdecoder: logdecoder.o
	@echo "Linking $@$(STATIC_STR)..."
	@$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

%.o: %.cpp .FORCE
	@echo "Compiling $<..."
	@$(CXX) $(CXXFLAGS) -c $<

clean:
	rm logdecoder *.o

help:
	@echo "make        - build the logdecoder"
	@echo "make static - build the logdecoder with static linking (more portable)"
	@echo "make clean  - remove object files and binaries"
	@echo "make help   - display help info"
