# Host side unit tests for the pure controller logic in ps-controller/ps_logic.h.
#
#   make          build and run the tests
#   make compile  build only
#   make clean    remove build output
#
# Needs nothing but a C++17 compiler. On Windows, run it from WSL:
#   wsl make -C test

CXX       ?= g++
CXXFLAGS  ?= -std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Wshadow -Wconversion
INCLUDES   = -I. -I../ps-controller

BUILD_DIR := build
SOURCES   := $(wildcard *.cpp)
OBJECTS   := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SOURCES))
TARGET    := $(BUILD_DIR)/run_tests

.PHONY: all compile test clean

all: test

compile: $(TARGET)

test: $(TARGET)
	@./$(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

# Header changes must force a rebuild.
$(OBJECTS): test_framework.h ../ps-controller/ps_logic.h
