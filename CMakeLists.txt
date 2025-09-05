cmake_minimum_required(VERSION 3.16)
project(lowlat_multitrack CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-fno-rtti)

if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O3)
endif()

add_executable(multitrack cli/main.cpp)
target_include_directories(multitrack PUBLIC ${CMAKE_SOURCE_DIR})

target_link_libraries(multitrack PRIVATE pthread)