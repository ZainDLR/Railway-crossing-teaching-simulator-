CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra -Wpedantic -Werror
.PHONY: all test clean
all: crossing
crossing: src/main.cpp src/controller.cpp src/api.hpp
	$(CXX) $(CXXFLAGS) -pthread src/main.cpp src/controller.cpp -o $@
test: crossing
	python3 tests/test_scenarios.py
clean:
	rm -f crossing trace.csv
