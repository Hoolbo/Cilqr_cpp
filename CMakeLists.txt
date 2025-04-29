
cmake_minimum_required(VERSION 3.15)
project(Cilqr_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
add_compile_options(/utf-8)
endif()

find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
include_directories(${EIGEN3_INCLUDE_DIR})
else()
message(FATAL_ERROR "Eigen3 not found. Please install Eigen3.")
endif()

set(Python3_EXECUTABLE "/usr/bin/python3.8")
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
if(Python3_FOUND)
include_directories(${Python3_INCLUDE_DIRS})
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
set(PYTHON_LIBRARY ${Python3_LIBRARIES})
else()
message(FATAL_ERROR "Python3 not found. Please install Python3.")
endif()

add_library(matplotlib_cpp INTERFACE)
target_compile_features(matplotlib_cpp INTERFACE cxx_std_17)
target_include_directories(matplotlib_cpp INTERFACE ${Python3_INCLUDE_DIRS})
target_link_libraries(matplotlib_cpp INTERFACE ${Python3_LIBRARIES})

execute_process(
COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
RESULT_VARIABLE NUMPY_NOT_FOUND
OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NUMPY_NOT_FOUND EQUAL 0)
target_include_directories(matplotlib_cpp INTERFACE ${NUMPY_INCLUDE_DIR})
message(STATUS "NumPy found at ${NUMPY_INCLUDE_DIR}")
else()
target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
message(WARNING "NumPy not found. Compiling without NumPy support.")
endif()

include_directories(include)

if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-g   -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

aux_source_directory(./src SRC_LIST)

add_executable(main ${SRC_LIST})

if(WIN32)
target_link_libraries(main PRIVATE matplotlib_cpp python36_d.lib npymath.lib)
else()
target_link_libraries(main PRIVATE matplotlib_cpp ${Python3_LIBRARIES} pthread dl util)
endif()