cmake_minimum_required(VERSION 3.18)
project(Cilqr_cpp)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖库 - 暂时注释外部依赖以便测试
# find_package(OpenCV REQUIRED)
find_package(Eigen3 QUIET)
find_package(nlohmann_json QUIET)

message(STATUS "Building with full dependencies")

# 跨平台查找 Python3（解释器 + 开发头文件/库）
# - Linux/Windows 都可用，不再硬编码路径
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
# 可选的 NumPy（若找到则使用其包含目录）
find_package(Python3 COMPONENTS NumPy QUIET)

# 创建 matplotlib_cpp 接口库，用于封装对 Python 的依赖
add_library(matplotlib_cpp INTERFACE)
target_compile_features(matplotlib_cpp INTERFACE cxx_std_11)
# 通过导入目标提供包含目录与链接库，跨平台兼容
target_link_libraries(matplotlib_cpp INTERFACE Python3::Python)

# 若找到 NumPy，则加入其包含目录；否则定义 WITHOUT_NUMPY 以兼容头文件逻辑
if(Python3_NumPy_FOUND)
    target_include_directories(matplotlib_cpp INTERFACE ${Python3_NumPy_INCLUDE_DIRS})
    message(STATUS "NumPy found at ${Python3_NumPy_INCLUDE_DIRS}")
else()
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
    message(WARNING "NumPy not found. Compiling without NumPy support.")
endif()

# 包含项目头文件目录
include_directories(include)
# 为兼容某些环境下的 FindEigen3，保留包含目录（若使用 Eigen3::Eigen 目标可不需要）
if(EIGEN3_INCLUDE_DIR)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# 创建可执行文件 - 明确指定源文件
set(MAIN_SOURCES
    src/main.cpp
    src/ilqr.cpp
    src/utils.cpp
    # src/map_data.cpp  # 暂时排除，因为依赖OpenCV
)
add_executable(main ${MAIN_SOURCES})

# 坐标转换测试程序暂时禁用
# add_executable(test_coordinate_conversion src/test_coordinate_conversion.cpp)

# 链接库
target_link_libraries(main PRIVATE matplotlib_cpp)
# 暂时注释OpenCV链接以便测试
# target_link_libraries(main PRIVATE ${OpenCV_LIBS})
# 条件性链接 nlohmann_json
if(TARGET nlohmann_json::nlohmann_json)
    target_link_libraries(main PRIVATE nlohmann_json::nlohmann_json)
    message(STATUS "Using nlohmann_json library")
else()
    message(WARNING "nlohmann_json not found, using header-only fallback")
endif()
# 条件性链接 Eigen3
if(TARGET Eigen3::Eigen)
    target_link_libraries(main PRIVATE Eigen3::Eigen)
elseif(EIGEN3_INCLUDE_DIR)
    # 如果找到了 EIGEN3_INCLUDE_DIR，但没有目标，则仅包含目录（无需链接）
    message(STATUS "Using system Eigen3 from ${EIGEN3_INCLUDE_DIR}")
endif()
message(STATUS "Full linking restored")

# 测试程序不需要额外的库依赖

# Windows 专项小优化（可选）：避免 <windows.h> 与 std::min/max 冲突、启用数学常量
if(MSVC)
    target_compile_definitions(main PRIVATE NOMINMAX _USE_MATH_DEFINES)
endif()