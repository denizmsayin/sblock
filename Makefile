CXX=g++
CXXFLAGS=-Wall -std=c++17
EXTRAFLAGS?=-O3
BINPATH=bin/
IFLAGS=
LFLAGS=
# used to set the height and width for which the db generator is compiled
PH=3
PW=3
HW=-D__W=$(PW) -D__H=$(PH)

# to include torch
# seems to work fine, but having issues with ldconfig at runtime :(
TORCHPATH=/home/deniz/libtorch
TORCH=no
ifeq ($(TORCH),yes)
    IFLAGS=-I$(TORCHPATH)/include
    LFLAGS=-L$(TORCHPATH)/lib -ltorch -lc10 -lmkldnn -lgomp -lnvToolsExt
    LFLAGS+=-L/usr/local/cuda/lib64 -lcudnn -lcudart -lcublas -lcurand -lcusolver -lnvrtc
    LFLAGS+=-L/usr/local/cuda/lib64/stubs/ -lcuda
    EXTRAFLAGS+=-DW_TORCH
endif

PREFIX=$(CXX) $(IFLAGS) $(CXXFLAGS) $(EXTRAFLAGS)
POSTFIX=$(LFLAGS)

sbpuzzle_test: sbpuzzle_test.cpp sblock_utils.cpp
	$(PREFIX) $(HW) $^ -o $(BINPATH)$@_$(PH)x$(PW).out $(LFLAGS)

generate_dpdb: generate_dpdb.cpp sblock_utils.cpp
	$(PREFIX) $(HW) $^ -o $(BINPATH)$@_$(PH)x$(PW).out $(LFLAGS)

sbp_tests: sbp_tests.cpp sblock_utils.cpp
	$(PREFIX) $^ -o $(BINPATH)$@.out $(LFLAGS)

generate_solutions: generate_solutions.cpp sblock_utils.cpp
	$(PREFIX) $(HW) $^ -o $(BINPATH)$@_$(PH)x$(PW).out -lpthread $(LFLAGS)

cmp_solution_files: cmp_solution_files.cpp sblock_utils.cpp
	$(PREFIX) $^ -o $(BINPATH)$@.out $(LFLAGS)

check_solution_file: check_solution_file.cpp sblock_utils.cpp
	$(PREFIX) $(HW) $^ -o $(BINPATH)$@_$(PH)x$(PW).out $(LFLAGS)

train_davi: train_davi.cpp sblock_utils.cpp
	$(PREFIX) $(HW) $^ -o $(BINPATH)$@_$(PH)x$(PW).out $(LFLAGS)

