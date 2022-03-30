#
# (c) Bit Parallel Ltd (Max van Daalen), March 2022
#

CC=g++

CC_COMPILE_FLAGS=-std=c++17 -O3 -I .
CC_LINK_FLAGS=-pthread

all: serial-port.o test.o
	$(CC) $(CC_LINK_FLAGS) serial-port.o test.o -o test

serial-port.o: serial-port.cpp
	$(CC) $(CC_COMPILE_FLAGS) -c serial-port.cpp

test.o: test.cpp
	$(CC) $(CC_COMPILE_FLAGS) -c test.cpp

.PHONY clean:
clean:
	rm -f *.o test
