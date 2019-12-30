cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(sblock)

option(WITH_TORCH "if enabled, code is compiled with TorchScript support" ON)

set(THREAD_LIB "-pthread")

if(WITH_TORCH)
    find_package(Torch REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

function(add_target_to_build_with_libs TRG SRCLIST LDFLAGS D)
    add_executable("${TRG}" ${SRCLIST})
    target_link_libraries("${TRG}" ${LDFLAGS})
    target_compile_definitions("${TRG}" PRIVATE __H=${D} __W=${D})
    if(WITH_TORCH)
        target_link_libraries("${TRG}" "${TORCH_LIBRARIES}")
    endif()
    set_property(TARGET "${TRG}" PROPERTY CXX_STANDARD 17)
endfunction()

function(add_target_to_build TRG SRCLIST)
    add_target_to_build_with_libs("${TRG}" "${SRCLIST}" "" "")
endfunction()

foreach(D 3 4 5)
    set(SRCLIST "solver.cpp" "sblock_utils.cpp")
    add_target_to_build_with_libs("solver_${D}x${D}" "${SRCLIST}" "-pthread" "${D}")
endforeach()

foreach(D 3 4 5)
    set(SRCLIST "generate_dpdb.cpp" "sblock_utils.cpp")
    add_target_to_build_with_libs("generate_dpdb_${D}x${D}" "${SRCLIST}" "" "${D}")
endforeach()

foreach(T tests_basic tests_distrs cmp_solution_files tests_gen)
    add_target_to_build("${T}" "${T}.cpp;sblock_utils.cpp")
endforeach()
