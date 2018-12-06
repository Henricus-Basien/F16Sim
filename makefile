#***************************************
# Settings
#***************************************

EXE = F16Sim

CC=gcc
EXT=c

#+++++++++++++++++++++
# Paths
#+++++++++++++++++++++

SRC_DIR=src
OBJ_DIR=obj

#----------------------------------------
# Create Paths
#----------------------------------------

MKDIR_P = mkdir -p

#+++++++++++++++++++++
# Source Files
#+++++++++++++++++++++

SRC = $(wildcard $(SRC_DIR)/*.$(EXT))
OBJ = $(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

#+++++++++++++++++++++
# Flags
#+++++++++++++++++++++

# Compiler Flags
CFLAGS   += -Wall -Wno-implicit-function-declaration -Wno-unused-variable -Wno-unused-but-set-variable -Wno-int-conversion -Wno-comment # some warnings about bad code
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

$(EXE): $(OBJ)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

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