CXX=g++
CXXFLAGS=-Wall -std=c++17
EXTRAFLAGS?=-O3
SRC=sbpuzzle_test.cpp sblock_utils.cpp
OUT=sbpuzzle_test.out
SRC2=generate_dpdb.cpp sblock_utils.cpp
OUT2=generate_dpdb.out
TESTSRC=sbp_tests.cpp sblock_utils.cpp
TESTOUT=sbp_tests.out

# CXXFLAGS += -DTRACK_NODES

all:
	$(CXX) $(CXXFLAGS) $(EXTRAFLAGS) $(SRC) -o $(OUT)

dpdb:
	$(CXX) $(CXXFLAGS) $(EXTRAFLAGS) $(SRC2) -o $(OUT2)

tests:
	$(CXX) $(CXXFLAGS) $(EXTRAFLAGS) $(TESTSRC) -o $(TESTOUT)
