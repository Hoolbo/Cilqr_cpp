cmake_minimum_required(VERSION 3.15)  
project(main)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(/utf-8)


find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED)
find_package(Matlab REQUIRED)
# find_package(Numpy REQUIRED)
# find_package(matplotlibcpp REQUIRED)

set(Eigen3_INCLUDE_DIRS "C:/Users/Hoolbo/Code/Tools/vcpkg/installed/x64-windows/include")

set(Python3_INCLUDE_DIRS "C:/Users/Hoolbo/AppData/Local/Programs/Python/Python36/include")
set(PYTHON3_LIBRARY "C:/Users/Hoolbo/AppData/Local/Programs/Python/Python36/libs")

set(Matlab_INCLUDE_DIRS "C:/Program Files/MATLAB/R2023b/extern/include")
set(Matlab_LIBRARY "C:/Program Files/MATLAB/R2023b/extern/lib/win64/microsoft")

set(Numpy_INCLUDE_DIRS "C:/Users/Hoolbo/AppData/Local/Programs/Python/Python36/Lib/site-packages/numpy/core/include")
set(Numpy_LIBRARY "C:/Users/Hoolbo/AppData/Local/Programs/Python/Python36/Lib/site-packages/numpy/core/lib")

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${Matlab_INCLUDE_DIRS}
    ${Numpy_INCLUDE_DIRS}
)
link_directories(
    libs
    ${PYTHON3_LIBRARY}
    ${Matlab_LIBRARY}
    ${Numpy_LIBRARY}
)

# # 优化配置
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# 目标配置
aux_source_directory(./src SRC_LIST)
add_executable(main ${SRC_LIST})
# add_executable(main test.cpp)

# 优化选项
target_compile_options(main PRIVATE
    $<$<CONFIG:RELEASE>:-O3>
    $<$<CXX_COMPILER_ID:GNU,Clang>:-march=native>
    $<$<CONFIG:RELEASE>:-flto>
)

# LTO支持
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
target_link_options(main PRIVATE
    $<$<CONFIG:RELEASE>:-flto>
)

# MSVC特殊处理
if(MSVC)
    target_compile_options(main PRIVATE /O2 /arch:AVX2 /GL)
    target_link_options(main PRIVATE /LTCG)
endif()



target_link_libraries(main PRIVATE 
    mclmcrrt.lib
    python36_d.lib
    npymath.lib
)
