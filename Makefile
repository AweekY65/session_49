# rigid2d — terminal-only build & test.
# Requires only a C++17 compiler (g++/clang++); no external libraries.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS ?= -Iinclude -Itests

SRC      := $(wildcard src/*.cpp)
TESTS    := $(wildcard tests/test_*.cpp)
OBJ      := $(SRC:.cpp=.o)

BUILD    := build
TEST_BIN := $(BUILD)/rigid2d_tests
DEMO_BIN := $(BUILD)/headless_demo

.PHONY: all test demo clean

all: $(TEST_BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(TEST_BIN): $(SRC) $(TESTS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(SRC) $(TESTS) -o $@

$(DEMO_BIN): $(SRC) examples/headless_demo.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(SRC) examples/headless_demo.cpp -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

demo: $(DEMO_BIN)
	./$(DEMO_BIN)

clean:
	rm -rf $(BUILD) $(OBJ)
