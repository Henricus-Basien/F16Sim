EXE = F16Sim

ifdef OS
    #EXE +=.exe
    RM = del /Q
    FixPath = $(subst /,\,$1)
    MKDIR_P = mkdir
else
    ifeq ($(shell uname), Linux)
    	
        RM = rm -f
        FixPath = $1
        MKDIR_P = mkdir -p
    endif
endif

#$(call FixPath,TEXT)

#***************************************
# Settings
#***************************************

CC=gcc
EXT=c

#+++++++++++++++++++++
# Paths
#+++++++++++++++++++++

SRC_DIR=src
OBJ_DIR=obj

#+++++++++++++++++++++
# Source Files
#+++++++++++++++++++++

SRC = $(wildcard $(SRC_DIR)/*.$(EXT))
OBJ = $(SRC:$(SRC_DIR)/%.$(EXT)=$(OBJ_DIR)/%.o)

#+++++++++++++++++++++
# Flags
#+++++++++++++++++++++

# Compiler Flags
#STD = -std=gnu11
STD = -std=gnu99
#STD = -std=c99
CFLAGS   += $(STD) -Wall -Wno-implicit-function-declaration -Wno-unused-variable -Wno-unused-but-set-variable -Wno-int-conversion -Wno-comment # some warnings about bad code
# Preprocessor Flags
CPPFLAGS += -Iinclude # -I is a preprocessor flag, not a compiler flag
# Linker Flags
LDFLAGS  += -Llib
# External Libraries
LDLIBS   += -lm

#***************************************
# Compile
#***************************************

.PHONY: all clean directories

all: directories $(EXE)
# all:
# 	@echo "SRC: $(SRC)"
# 	@echo "OBJ: $(OBJ)"

$(EXE): $(OBJ)
	$(CC) $(LDFLAGS) $(call FixPath,$^) $(LDLIBS) -o $(call FixPath,$@)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $(call FixPath,$<) -o $(call FixPath,$@)

#***************************************
# Clean
#***************************************

clean:
	$(RM) -r $(OBJ_DIR)/*

#***************************************
# MakeDirectories
#***************************************

directories: ${OBJ_DIR}

${OBJ_DIR}:
	${MKDIR_P} ${OBJ_DIR}