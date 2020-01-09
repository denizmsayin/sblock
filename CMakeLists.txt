cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(sblock)

option(WITH_TORCH "if enabled, code is compiled with TorchScript support" ON)
option(WITH_TESTS "if enabled, test source files are also compiled" OFF)

set(THREAD_LIB "-pthread")

if(WITH_TORCH)
    find_package(Torch REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

function(add_target_to_build_with_libs TRG SRCLIST LDFLAGS D IS_LIB)
    if(${IS_LIB})
        add_library("${TRG}" SHARED ${SRCLIST})
    else()
        add_executable("${TRG}" ${SRCLIST})
    endif()
    target_link_libraries("${TRG}" ${LDFLAGS})
    target_compile_definitions("${TRG}" PRIVATE __H=${D} __W=${D})
    if(WITH_TORCH)
        target_link_libraries("${TRG}" "${TORCH_LIBRARIES}")
        target_compile_definitions("${TRG}" PRIVATE W_TORCH)
    endif()
    set_property(TARGET "${TRG}" PROPERTY CXX_STANDARD 17)
endfunction()

function(add_target_to_build TRG SRCLIST)
    add_target_to_build_with_libs("${TRG}" "${SRCLIST}" "" "" FALSE)
endfunction()

if(FALSE)
foreach(D 3 4 5)
    # add the solver
    set(SRCLIST "solver.cpp" "utils/basic.cpp")
    add_target_to_build_with_libs("solver_${D}x${D}" "${SRCLIST}" "-pthread" "${D}" FALSE)
    # add the pdb generator
    set(SRCLIST "generate_dpdb.cpp" "utils/basic.cpp")
    add_target_to_build_with_libs("generate_dpdb_${D}x${D}" "${SRCLIST}" "" "${D}" FALSE)
    # add the data generator as a shared library
    set(SRCLIST "libdig.cpp" "utils/basic.cpp")
    add_target_to_build_with_libs("dig_${D}x${D}" "${SRCLIST}" "" "${D}" TRUE)
endforeach()
endif()

# remaining misc. programs
if(WITH_TESTS)
    # set(EXTRAS "cmp_solution_files;tests_basic;tests_distrs;tests_gen")
    set(EXTRAS "tests_basic")
endif()

foreach(T ${EXTRAS})
    add_target_to_build("${T}" "tests/${T}.cpp;utils/basic.cpp")
endforeach()
