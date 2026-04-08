# Sensor Fusion System — Build Configuration
# Requires: g++ or clang++ with C++17, SFML 3.x (Homebrew on Mac, libsfml-dev on Linux)
#
# Usage:
#   make          - Build the simulation with GUI
#   make run      - Build and run
#   make clean    - Remove build artifacts

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2

# Auto-detect Homebrew SFML on macOS
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  # Apple Silicon Homebrew
  ifneq ($(wildcard /opt/homebrew/include/SFML/.),)
    CXXFLAGS += -I/opt/homebrew/include
    LDFLAGS  += -L/opt/homebrew/lib
  endif
  # Intel Homebrew
  ifneq ($(wildcard /usr/local/include/SFML/.),)
    CXXFLAGS += -I/usr/local/include
    LDFLAGS  += -L/usr/local/lib
  endif
endif

SFML_LIB = -lsfml-graphics -lsfml-window -lsfml-system
MATH_LIB = -lm

SRC_DIR = src
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
TARGET  = simulation

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCES) $(LDFLAGS) $(SFML_LIB) $(MATH_LIB)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
