CFLAGS=-Wall -std=c++11 -O0
SRC=sbpuzzle.cpp sbpuzzle_test.cpp
OUT=sbpuzzle_test.out

all:
	$(CXX) $(CFLAGS) $(SRC) -o $(OUT)
