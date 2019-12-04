CXX=g++
CXXFLAGS=-Wall -std=c++17
RELEASEFLAGS=-O3
DEBUGFLAGS=-O0 -g
SRC=sbpuzzle_test.cpp sblock_utils.cpp
OUT=sbpuzzle_test.out
SRC2=generate_dpdb.cpp sblock_utils.cpp
OUT2=generate_dpdb.out
TESTSRC=sbp_tests.cpp sblock_utils.cpp
TESTOUT=sbp_tests.out

# CXXFLAGS += -DTRACK_NODES

all:
	$(CXX) $(CXXFLAGS) $(RELEASEFLAGS) $(SRC) -o $(OUT)

debug:
	$(CXX) $(CXXFLAGS) $(DEBUGFLAGS) $(SRC) -o $(OUT)

dpdb:
	$(CXX) $(CXXFLAGS) $(RELEASEFLAGS) $(SRC2) -o $(OUT2)

dpdebug:
	$(CXX) $(CXXFLAGS) $(DEBUGFLAGS) $(SRC2) -o $(OUT2)

tests:
	$(CXX) $(CXXFLAGS) $(RELEASEFLAGS) $(TESTSRC) -o $(TESTOUT)
