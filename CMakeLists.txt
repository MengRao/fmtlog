cmake_minimum_required(VERSION 3.15)

project(fmtlog CXX)

#SET(CMAKE_CXX_COMPILER "g++")

add_compile_options(-Wall -Werror -O3 -std=c++17)

link_directories(.)
include_directories(fmt/include)
link_libraries(pthread)

add_library(fmtlog-shared SHARED fmtlog.cc)
#target_link_libraries(fmtlog-shared)
install(TARGETS fmtlog-shared)

add_library(fmtlog-static fmtlog.cc)
#target_link_libraries(fmtlog-static)
install(TARGETS fmtlog-static)

add_subdirectory(fmt)
add_subdirectory(test)
add_subdirectory(bench)
