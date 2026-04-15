# ============================================================
# Makefile for Universe Simulator
# ============================================================
# Linux:   make
# Windows: mingw32-make -f Makefile

CC      = gcc
TARGET  = universe

# Source files
SRC = universe.c

# Compiler flags
CFLAGS  = -Wall -Wextra -O2 -std=c99

# Auto-detect platform
UNAME := $(shell uname -s 2>/dev/null || echo Windows)

ifeq ($(UNAME), Linux)
    # Linux
    LDFLAGS = -lSDL2 -lSDL2_ttf -lGL -lGLU -lm
    EXT     =
else ifeq ($(UNAME), Darwin)
    # macOS (SDL2 via Homebrew, frameworks for GL)
    CFLAGS  += $(shell sdl2-config --cflags)
    LDFLAGS  = $(shell sdl2-config --libs) -lSDL2_ttf -framework OpenGL -lm
    EXT     =
else
    # Windows (MSYS2 / MinGW)
    SDL2_DIR ?= C:/SDL2
    CFLAGS  += -I$(SDL2_DIR)/include/SDL2
    LDFLAGS  = -L$(SDL2_DIR)/lib -lSDL2 -lSDL2_ttf -lopengl32 -lglu32 -lm
    EXT     = .exe
endif

all: $(TARGET)$(EXT)

$(TARGET)$(EXT): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET) $(TARGET).exe

.PHONY: all clean