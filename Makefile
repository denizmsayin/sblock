CXX=clang++
CFLAGS=-Wall -std=c++17
RELEASEFLAGS=-O3
DEBUGFLAGS=-O0 -g
SRC=sbpuzzle_test.cpp sblock_utils.cpp
OUT=sbpuzzle_test.out
SRC2=generate_dpdb.cpp sblock_utils.cpp
OUT2=generate_dpdb.out

all:
	$(CXX) $(CFLAGS) $(RELEASEFLAGS) $(SRC) -o $(OUT)

debug:
	$(CXX) $(CFLAGS) $(DEBUGFLAGS) $(SRC) -o $(OUT)

dpdb:
	$(CXX) $(CFLAGS) $(DEBUGFLAGS) $(SRC2) -o $(OUT2)
