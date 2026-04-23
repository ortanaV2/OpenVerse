# ============================================================
# Makefile — verse solar system simulator
# ============================================================
# Linux:   make
# Windows: mingw32-make  (MSYS2 / MinGW-w64)

CC      = gcc
TARGET  = verse

SRCDIR  = src
SRCS    = $(wildcard $(SRCDIR)/*.c)
OBJS    = $(SRCS:.c=.o)

CFLAGS  = -Wall -Wextra -O2 -std=c99 -I$(SRCDIR) -fopenmp

# Auto-detect platform
UNAME := $(shell uname -s 2>/dev/null || echo Windows)

ifeq ($(UNAME), Linux)
    CFLAGS  += $(shell sdl2-config --cflags)
    LDFLAGS  = $(shell sdl2-config --libs) -lSDL2_ttf -lGL -lGLEW -lm -fopenmp
    EXT      =

else ifeq ($(UNAME), Darwin)
    CFLAGS  += $(shell sdl2-config --cflags)
    LDFLAGS  = $(shell sdl2-config --libs) -lSDL2_ttf -lGLEW \
               -framework OpenGL -lm -fopenmp
    EXT      =

else
    # Windows — MSYS2 / MinGW-w64
    # Set SDL2_DIR if SDL2 is not in the default MinGW prefix.
    # Example: SDL2_DIR = C:/msys64/mingw64
    SDL2_DIR ?= C:/msys64/mingw64
    CFLAGS  += -I$(SDL2_DIR)/include/SDL2 -I$(SDL2_DIR)/include
    LDFLAGS  = -L$(SDL2_DIR)/lib \
               -lSDL2 -lSDL2_ttf \
               -lglew32 \
               -lopengl32 -lglu32 \
               -lm -fopenmp
    EXT      = .exe
endif

# ---- Rules ---------------------------------------------------------
all: $(TARGET)$(EXT)

$(TARGET)$(EXT): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(SRCDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(SRCDIR)/*.o $(TARGET) $(TARGET).exe

.PHONY: all clean
