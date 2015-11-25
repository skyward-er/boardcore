##
## Makefile for Miosix embedded OS
##
MAKEFILE_VERSION := 1.04
## Path to kernel directory (edited by init_project_out_of_git_repo.pl)
KPATH := libs/miosix-kernel/miosix
## Path to config directory (edited by init_project_out_of_git_repo.pl)
CONFPATH := .
include $(CONFPATH)/config/Makefile.inc

##
## List here subdirectories which contains makefiles
##
SUBDIRS := $(KPATH) 

##
## List here your source files (both .s, .c and .cpp)
##
SRC := \
	src/shared/canbus/CanManager.cpp \
	src/shared/canbus/CanSocket.cpp \
	src/main.cpp

##
## List here additional static libraries with relative path
##
LIBS := 

##
## List here additional include directories (in the form -Iinclude_dir)
##
INCLUDE_DIRS := -Isrc/shared -Imiosix-kernel -Imiosix

##############################################################################
## You should not need to modify anything below                             ##
##############################################################################

## Replaces both "foo.cpp"-->"foo.o" and "foo.c"-->"foo.o"
OBJ := $(addsuffix .o, $(basename $(SRC)))

## Includes the miosix base directory for C/C++
## Always include CONFPATH first, as it overrides the config file location
CXXFLAGS := $(CXXFLAGS_BASE) -I$(CONFPATH) -I$(CONFPATH)/config/$(BOARD_INC)  \
            -I. -I$(KPATH) -I$(KPATH)/arch/common -I$(KPATH)/$(ARCH_INC)      \
            -I$(KPATH)/$(BOARD_INC) $(INCLUDE_DIRS) -std=c++11 -O0
CFLAGS   := $(CFLAGS_BASE)   -I$(CONFPATH) -I$(CONFPATH)/config/$(BOARD_INC)  \
            -I. -I$(KPATH) -I$(KPATH)/arch/common -I$(KPATH)/$(ARCH_INC)      \
            -I$(KPATH)/$(BOARD_INC) $(INCLUDE_DIRS)
AFLAGS   := $(AFLAGS_BASE)
LFLAGS   := $(LFLAGS_BASE)
DFLAGS   := -MMD -MP
BIN 	 := bin

LINK_LIBS := $(LIBS) -L$(KPATH) -Wl,--start-group -lmiosix -lstdc++ -lc \
             -lm -lgcc -Wl,--end-group

all: folders all-recursive main

clean: clean-recursive clean-topdir

program:
	$(PROGRAM_CMDLINE)

all-recursive:
	$(foreach i,$(SUBDIRS),$(MAKE) -C $(i)                               \
	  KPATH=$(shell perl $(KPATH)/_tools/relpath.pl $(i) $(KPATH))       \
	  CONFPATH=$(shell perl $(KPATH)/_tools/relpath.pl $(i) $(CONFPATH)) \
	  || exit 1;)

clean-recursive:
	$(foreach i,$(SUBDIRS),$(MAKE) -C $(i)                               \
	  KPATH=$(shell perl $(KPATH)/_tools/relpath.pl $(i) $(KPATH))       \
	  CONFPATH=$(shell perl $(KPATH)/_tools/relpath.pl $(i) $(CONFPATH)) \
	  clean || exit 1;)

clean-topdir:
	rm -f $(OBJ) $(addprefix obj/,$(OBJ)) \
		$(BIN)/main.elf $(BIN)/main.hex $(BIN)/main.bin $(BIN)/main.map \
		$(OBJ:.o=.d)

main: folders main.elf
	$(CP) -O ihex   $(BIN)/main.elf $(BIN)/main.hex
	$(CP) -O binary $(BIN)/main.elf $(BIN)/main.bin
	$(SZ) $(BIN)/main.elf

main.elf: folders $(OBJ) all-recursive
	@ echo "linking"
	$(CXX) $(LFLAGS) -o $(BIN)/main.elf $(addprefix obj/,$(OBJ)) $(KPATH)/$(BOOT_FILE) $(LINK_LIBS)

%.o: %.s
	@ echo "[AS]  " $<
	@ $(AS)  $(AFLAGS) $< -o $@

%.o : %.c
	@ echo "[CC]  " $<
	@ $(CC)  $(DFLAGS) $(CFLAGS) $< -o obj/$@

%.o : %.cpp
	@ echo "[CXX] " $<
	@ $(CXX) $(DFLAGS) $(CXXFLAGS) $< -o obj/$@

folders:
	mkdir -pv $(dir $(addprefix obj/,$(OBJ)))

#pull in dependecy info for existing .o files
-include $(OBJ:.o=.d)
