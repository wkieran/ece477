CC=g++
CFLAGS=-std=c++17 -Wall -g
DFLAGS=

# The source files we use for building custom_tests
ALL_SRC=$(wildcard *.cpp)

# The name of the resulting executable
APP=test

ifeq ($(part1), on)
	DFLAGS += -DPART1
endif

ifeq ($(part2), on)
	DFLAGS += -DPART2
endif

sketch: sketch.cpp
	$(CC) $(CFLAGS) $^ -o sketch
	./sketch

trace_%.cpp : trace_%.txt
	python3 trace_files/compile_trace.py $<

test: $(ALL_SRC)
	$(CC) $(CFLAGS) $^ -o main
	./main

clean:
	rm -f $(APP)
