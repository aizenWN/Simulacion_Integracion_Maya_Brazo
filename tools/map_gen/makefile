$(info Random Map Generator build starts...)

CC := g++
CC_OPTS := -O3

# Get OpenCV4 include and library flags from pkg-config
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS   := $(shell pkg-config --libs opencv4)

SRCS    := $(wildcard src/*.cpp)
HEADERS := $(wildcard include/*.h)
OBJECTS := $(patsubst src/%.cpp,build/%.o,$(SRCS))

# Default target
all: build/map_generator

build/map_generator: $(OBJECTS)
	$(CC) $(OBJECTS) $(CC_OPTS) $(OPENCV_LIBS) -o $@
	$(info Random Map Generator is successfully built!)

build/%.o: src/%.cpp $(HEADERS)
	$(CC) $(CC_OPTS) $(OPENCV_CFLAGS) -c $< -o $@

.PHONY: all

