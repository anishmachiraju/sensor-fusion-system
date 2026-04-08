# Sensor Fusion System - Build Configuration
# Requires: g++ or clang++ with C++17, SFML 3.x
#
# Usage:
#   make             - Build the simulation (links SFML for GUI mode)
#   make run         - Build and run in GUI mode (default)
#   make terminal    - Build and run in terminal mode (no GUI)
#   make gui         - Build and run in GUI mode (explicit)
#   make clean       - Remove build artifacts

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2

# Auto-detect Homebrew SFML on macOS
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  ifneq ($(wildcard /opt/homebrew/include/SFML/.),)
    CXXFLAGS += -I/opt/homebrew/include
    LDFLAGS  += -L/opt/homebrew/lib
  endif
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

gui: $(TARGET)
	./$(TARGET) --gui

terminal: $(TARGET)
	./$(TARGET) --terminal

clean:
	rm -f $(TARGET)

.PHONY: all run gui terminal clean
