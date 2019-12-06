CXX=g++
CXXFLAGS=-Wall -std=c++17
EXTRAFLAGS?=-O3
BINPATH=bin/
# used to set the height and width for which the db generator is compiled
PH=3
PW=3
HW=-D__W=$(PW) -D__H=$(PH)

# CXXFLAGS += -DTRACK_NODES

PREFIX=$(CXX) $(CXXFLAGS) $(EXTRAFLAGS)

sbpuzzle_test: sbpuzzle_test.cpp sblock_utils.cpp
	$(PREFIX) $^ -o $(BINPATH)$@.out

generate_dpdb: generate_dpdb.cpp sblock_utils.cpp
	$(PREFIX) $^ -o $(BINPATH)$@_$(PH)x$(PW).out

sbp_tests: sbp_tests.cpp sblock_utils.cpp
	$(PREFIX) $^ -o $(BINPATH)$@.out

generate_solutions: generate_solutions.cpp sblock_utils.cpp
	$(PREFIX) $^ -o $(BINPATH)$@_$(PH)x$(PW).out -lpthread

cmp_solution_files: cmp_solution_files.cpp sblock_utils.cpp
	$(PREFIX) $^ -o $(BINPATH)$@.out

