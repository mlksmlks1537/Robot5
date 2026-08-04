cmake_minimum_required(VERSION 3.10)
project(Robot5)

# 设置 C++ 标准为 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 添加可执行文件
add_executable(Robot5 src/main.cpp src/Robot.cpp)
set_target_properties(Robot5 PROPERTIES VS_STARTUP_PROJECT TRUE)

# ---------- 新增：Eigen3 查找与集成 ----------
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found: ${Eigen3_VERSION}")
    target_link_libraries(Robot5 Eigen3::Eigen)   # 自动添加包含路径
else()
    find_path(EIGEN3_INCLUDE_DIR "Eigen/Dense"
        HINTS
            "$ENV{EIGEN3_DIR}"
            "C:/Program Files (x86)/eigen-5.0.0"
            "C:/local/include"
            "/usr/include/eigen3"
    )
    if(EIGEN3_INCLUDE_DIR)
        message(STATUS "Eigen3 include found at: ${EIGEN3_INCLUDE_DIR}")
        target_include_directories(Robot5 PRIVATE ${EIGEN3_INCLUDE_DIR})
    else()
        message(FATAL_ERROR "Eigen3 not found. Install Eigen or set EIGEN3_DIR environment variable to the Eigen include path.")
    endif()
endif()
# ---------- Eigen3 集成结束 ----------

# 头文件包含目录（DLL 文件夹）
target_include_directories(Robot5 PRIVATE ${CMAKE_SOURCE_DIR}/DLL)

# 使用 find_library 查找静态库/导入库（如果找不到会在配置时报错）
find_library(MCSLPCI_LIB MCSLPCI_PMC PATHS ${CMAKE_SOURCE_DIR}/DLL REQUIRED)
find_library(PMCPCI_LIB PMCPCIDrv   PATHS ${CMAKE_SOURCE_DIR}/DLL REQUIRED)

# 链接库
target_link_libraries(Robot5 ${MCSLPCI_LIB} ${PMCPCI_LIB})

# 预处理器定义
target_compile_definitions(Robot5 PRIVATE _CONSOLE)

# MSVC 特定编译选项
if(MSVC)
    target_compile_options(Robot5 PRIVATE /UTF-8)
endif()

# ------------------------------
# 在构建后自动复制 DLL 到输出目录
# ------------------------------
set(DLL_DIR ${CMAKE_SOURCE_DIR}/DLL)

set(DLL_FILES
    ${DLL_DIR}/MCSLPCI_PMC.dll
    ${DLL_DIR}/PMCPCIDrv.dll
)

foreach(DLL_FILE ${DLL_FILES})
    if(EXISTS ${DLL_FILE})
        add_custom_command(TARGET Robot5 POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${DLL_FILE}
                $<TARGET_FILE_DIR:Robot5>
            COMMENT "Copying ${DLL_FILE} to output directory"
        )
    else()
        message(WARNING "DLL file ${DLL_FILE} not found, will not be copied.")
    endif()
endforeach()