CXX=g++
CXXFLAGS=-Wall -std=c++17
EXTRAFLAGS?=-O3
SRC=sbpuzzle_test.cpp sblock_utils.cpp
OUT=sbpuzzle_test.out
TESTSRC=sbp_tests.cpp sblock_utils.cpp
TESTOUT=sbp_tests.out

# used to set the height and width for which the db generator is compiled
PH=3
PW=3
HW=-D__W=$(PW) -D__H=$(PH)
DBGENSRC=generate_dpdb.cpp sblock_utils.cpp
DBGENOUT=generate_dpdb_$(PH)x$(PW).out
# CXXFLAGS += -DTRACK_NODES

all:
	$(CXX) $(CXXFLAGS) $(EXTRAFLAGS) $(SRC) -o $(OUT)

dpdb:
	$(CXX) $(CXXFLAGS) $(EXTRAFLAGS) $(HW) $(DBGENSRC) -o $(DBGENOUT)

tests:
	$(CXX) $(CXXFLAGS) $(EXTRAFLAGS) $(TESTSRC) -o $(TESTOUT)
