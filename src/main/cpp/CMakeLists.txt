cmake_minimum_required(VERSION 3.14)

# Avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP in CMake 3.24:
if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

# options
option(BUILD_TESTS "Build test programs" OFF)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# source files
file(GLOB_RECURSE MAIN_SRC
  algorithms/*.[ch]pp
  ds/*.[ch]pp
  generators/*.[ch]pp
  modular/*.[ch]pp
  readwrite/*.[ch]pp
  sat/*.[ch]pp
  util/*.[ch]pp
  external/*.[ch]pp
)

# external C sources
file(GLOB KISSAT_SRC external/kissat/*.c)
set_source_files_properties( ${KISSAT_SRC} PROPERTIES LANGUAGE C )

file(GLOB EXACT_SOLVER_SRC exact-solver.cpp)

include_directories(
  .
)

# project
project(tww)

# optimize kissat for UNsat instances
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG -DUNSAT -DNPROOFS -DQUIET")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -Wall -funroll-loops -fno-stack-limit -O3")

# build options
add_compile_definitions(PROFILE_ON=false)
add_compile_definitions(TRACE_ON=false)
add_compile_definitions(LOGGING_ON=false)

add_executable(exact-solver ${EXACT_SOLVER_SRC} ${KISSAT_SRC} ${MAIN_SRC})
