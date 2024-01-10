cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_STANDARD 20)

project(DownsampleLargePCD)

include(FetchContent)

FetchContent_Declare(fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 9.1.0
)
FetchContent_MakeAvailable(fmt)

find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (DownsampleLargePCD main.cpp)
target_link_libraries (DownsampleLargePCD ${PCL_LIBRARIES} fmt::fmt)
