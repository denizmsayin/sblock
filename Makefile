CFLAGS=-Wall -std=c++11
RELEASEFLAGS=-O3
DEBUGFLAGS=-O0 -g
SRC=sbpuzzle_test.cpp
OUT=sbpuzzle_test.out
SRC2=inversion_test.cpp
OUT2=inversion_test.out

all:
	$(CXX) $(CFLAGS) $(RELEASEFLAGS) $(SRC) -o $(OUT)

debug:
	$(CXX) $(CFLAGS) $(DEBUGFLAGS) $(SRC) -o $(OUT)

inv:
	$(CXX) $(CFLAGS) $(DEBUGFLAGS) $(SRC2) -o $(OUT2)
