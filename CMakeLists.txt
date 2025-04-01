cmake_minimum_required(VERSION 3.15)  
project(main)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(/utf-8)


find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED)
# find_package(Numpy REQUIRED)
# find_package(matplotlibcpp REQUIRED)

set(Eigen3_INCLUDE_DIRS "C:/Users/Hoolbo/Code/Tools/vcpkg/installed/x64-windows/include")

set(Python3_INCLUDE_DIRS "C:/Users/Hoolbo/AppData/Local/Programs/Python/Python36/include")
set(PYTHON3_LIBRARY "C:/Users/Hoolbo/AppData/Local/Programs/Python/Python36/libs")

set(Numpy_INCLUDE_DIRS "C:/Users/Hoolbo/AppData/Local/Programs/Python/Python36/Lib/site-packages/numpy/core/include")
set(Numpy_LIBRARY "C:/Users/Hoolbo/AppData/Local/Programs/Python/Python36/Lib/site-packages/numpy/core/lib")

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${Numpy_INCLUDE_DIRS}
)
link_directories(
    libs
    ${PYTHON3_LIBRARY}
    ${Numpy_LIBRARY}
)

# 优化配置
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
# 
set(CMAKE_CXX_FLAGS_DEBUG "-O0 ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# 目标配置
aux_source_directory(./src SRC_LIST)
add_executable(main ${SRC_LIST})

target_link_libraries(main PRIVATE 
    python36_d.lib
    npymath.lib
)
