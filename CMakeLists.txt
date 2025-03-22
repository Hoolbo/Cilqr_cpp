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

aux_source_directory(./src SRC_LIST)
add_executable(main ${SRC_LIST})

target_link_libraries(main PRIVATE 
    mclmcrrt.lib
    python36_d.lib
    npymath.lib
)
