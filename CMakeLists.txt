cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Only)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 递归收集所有用户源文件（以后加新文件不用改这里）
file(GLOB_RECURSE SOURCES
    "BSP/*.c"
    "BSP/*.cpp"
    "APP/*.c"
    "APP/*.cpp"
    "Algorithm/*.c"
    "Algorithm/*.cpp"
    "Module/*.c"
    "Module/*.cpp"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${SOURCES}
)

# 递归添加头文件路径：把目录及其所有子目录都加进 include 路径
function(include_sub_directories_recursively root_dir)
    if(IS_DIRECTORY ${root_dir})
        include_directories(${root_dir})
    endif()
    file(GLOB children LIST_DIRECTORIES true ${root_dir}/*)
    foreach(child ${children})
        if(IS_DIRECTORY ${child})
            include_sub_directories_recursively(${child})
        endif()
    endforeach()
endfunction()


# 递归添加头文件路径（以后新建子文件夹也不用改这里）
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/BSP)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/APP)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Algorithm)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Module)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)


# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
