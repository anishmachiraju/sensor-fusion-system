# Sensor Fusion System - Build Configuration
# Requires: g++ with C++17, SFML 2.5+
#
# Usage:
#   make          - Build the simulation with GUI
#   make run      - Build and run
#   make clean    - Remove build artifacts

CXX      = g++
CXXFLAGS = -std=c++17 -Wall -O2
SFML_LIB = -lsfml-graphics -lsfml-window -lsfml-system
MATH_LIB = -lm

SRC_DIR = src
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
TARGET  = simulation

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCES) $(SFML_LIB) $(MATH_LIB)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
