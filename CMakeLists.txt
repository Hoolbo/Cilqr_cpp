cmake_minimum_required(VERSION 3.10)
project(Cilqr_cpp)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 yaml-cpp
find_package(yaml-cpp REQUIRED)

# 查找 Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 显式指定 Python3.8 路径
set(PYTHON_EXECUTABLE "/usr/bin/python3.8")
set(PYTHON_INCLUDE_DIR "/usr/include/python3.8")
set(PYTHON_LIBRARY "/usr/lib/x86_64-linux-gnu/libpython3.8.so")

# 验证 Python3.8 路径
find_program(PYTHON_EXECUTABLE_FOUND ${PYTHON_EXECUTABLE})
if(NOT PYTHON_EXECUTABLE_FOUND)
    message(FATAL_ERROR "Python3.8 executable not found at ${PYTHON_EXECUTABLE}")
endif()

find_path(PYTHON_INCLUDE_DIR_FOUND Python.h ${PYTHON_INCLUDE_DIR})
if(NOT PYTHON_INCLUDE_DIR_FOUND)
    message(FATAL_ERROR "Python3.8 headers not found at ${PYTHON_INCLUDE_DIR}")
endif()

find_library(PYTHON_LIBRARY_FOUND python3.8 HINTS ${PYTHON_LIBRARY})
if(NOT PYTHON_LIBRARY_FOUND)
    message(FATAL_ERROR "Python3.8 library not found at ${PYTHON_LIBRARY}")
endif()

# 创建 matplotlib_cpp 接口库
add_library(matplotlib_cpp INTERFACE)
target_compile_features(matplotlib_cpp INTERFACE cxx_std_11)
target_include_directories(matplotlib_cpp INTERFACE ${PYTHON_INCLUDE_DIR})
target_link_libraries(matplotlib_cpp INTERFACE ${PYTHON_LIBRARY})

# 检查 NumPy（可选）
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    RESULT_VARIABLE NUMPY_NOT_FOUND
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NUMPY_NOT_FOUND EQUAL 0)
    target_include_directories(matplotlib_cpp INTERFACE ${NUMPY_INCLUDE_DIR})
    message(STATUS50 "NumPy found at ${NUMPY_INCLUDE_DIR}")
else()
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
    message(WARNING "NumPy not found. Compiling without NumPy support.")
endif()

# 包含项目头文件目录
include_directories(include)

# 收集源文件
aux_source_directory(./src SRC_LIST)

# 创建可执行文件
add_executable(main ${SRC_LIST})

# 链接库
target_link_libraries(main PRIVATE matplotlib_cpp yaml-cpp ${PYTHON_LIBRARY})