CFLAGS=-Wall -std=c++11
RELEASEFLAGS=-O3
DEBUGFLAGS=-O0 -g
SRC=sbpuzzle_test.cpp
OUT=sbpuzzle_test.out

all:
	$(CXX) $(CFLAGS) $(RELEASEFLAGS) $(SRC) -o $(OUT)

debug:
	$(CXX) $(CFLAGS) $(DEBUGFLAGS) $(SRC) -o $(OUT)
