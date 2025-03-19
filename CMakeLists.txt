    cmake_minimum_required(VERSION 3.10)
    project(main)

    set(CMAKE_CXX_STANDARD 17)
# vcpkg
    set(CMAKE_TOOLCHAIN_FILE "C:/Users/Hoolbo/Code/Tools/vcpkg/scripts/buildsystems/vcpkg.cmake")

#头文件目录
    include_directories(
        include
        "C:/Users/Hoolbo/Code/Tools/vcpkg/installed/x64-windows/include" #vcpkg头文件目录
        "C:/Program Files/MATLAB/R2023b/extern/include" #matlab头文件目录
    )

# 链接库目录
    link_directories(
        "C:/Program Files/MATLAB/R2023b/extern/lib/win64/microsoft"
    )
    find_package(Eigen3 REQUIRED)
    find_package(Matplot++ CONFIG REQUIRED)  
# 获取源文件列表
    aux_source_directory(./src SRC_LIST)

# 测试用
# add_executable(main test.cpp)  

# 创建可执行文件
    add_executable(main ${SRC_LIST})

# 正确链接目标库
    target_link_libraries(main PRIVATE 
        Eigen3::Eigen
        Matplot++::cimg  
        Matplot++::matplot  
        libmat.dll
        libmat.lib
        libmx.lib
        libmex.lib
        libeng.lib
        mclmcrrt.lib
    )