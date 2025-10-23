cmake_minimum_required(VERSION 3.23)

project(AMDAHL VERSION 0.0 LANGUAGES CXX)
set(AMDAHL_VERSION_PATCH 1)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GUROBI_ROOT_DIR "/home/yutinglu/gurobi1203/linux64" CACHE STRING "Specific the root of Gurobi")
option(USE_GUROBI "USE GUROBI" ON)

set(sources algo/lcs_srex/crossover.cc
        algo/lcs_srex/insert.cc
        algo/lcs_srex/lcs_dp.cc
        algo/lcs_srex/removal.cc
        algo/lcs_srex/route_subset.cc
        algo/lcs_srex/srex_sol.cc
        analysis/problem_analysis.cc
        analysis/solution_analysis.cc
        domain/model/entity/VrpRoute.cpp
        domain/model/VrpSolution.cpp
        domain/solver/hgs/ox.cpp
        domain/solver/memetic/crossover/EdgeAssemblyCrossover.cpp
        domain/solver/memetic/crossover/LcsDp.cpp
        domain/solver/memetic/crossover/RouteSubset.cpp
        domain/solver/memetic/crossover/SrexCrossover.cpp
        domain/solver/memetic/insert/Insert.cpp
        domain/solver/memetic/removal/Removal.cpp
        domain/solver/memetic/LargeNeighborhoodSearch.cpp
        domain/solver/memetic/LocalSearch.cpp
        domain/solver/memetic/Memetic.cpp
        domain/solver/memetic/Perturb.cpp
        io/pdptw_reader.cc
        io/sol_reader.cc
        io/sol_writer.cc
        io/vrp_sol_writer.cc
        model/solution.cc
        util/convert_utils.cc
        util/random_utils.cc
        util/sol_similarity.cc
)

if (${USE_GUROBI})
    add_compile_definitions(USE_GUROBI)
    set(sources ${sources} domain/solver/exact/TwoIndexFormulation.cpp)
    set(sources ${sources} domain/solver/exact/SetPartitioning.cpp)
    set(sources ${sources} domain/solver/exact/TwoIndChecker.cpp)
endif (${USE_GUROBI})

add_executable(amdahl
        ${sources}
        main.cc
)

configure_file(${AMDAHL_SOURCE_DIR}/config.h.in ${AMDAHL_BINARY_DIR}/config.h)

target_include_directories(amdahl PRIVATE ${AMDAHL_SOURCE_DIR} ${AMDAHL_BINARY_DIR}
)

target_include_directories(amdahl PRIVATE ${AMDAHL_SOURCE_DIR}/third_party)

if (${USE_GUROBI})
    target_include_directories(amdahl PRIVATE ${GUROBI_ROOT_DIR}/include/)
    target_link_libraries(amdahl PRIVATE ${GUROBI_ROOT_DIR}/lib/libgurobi_c++.a)
    target_link_libraries(amdahl PRIVATE ${GUROBI_ROOT_DIR}/lib/libgurobi120.so)
endif (${USE_GUROBI})

find_package(OpenMP REQUIRED)
target_link_libraries(amdahl PRIVATE OpenMP::OpenMP_CXX)
