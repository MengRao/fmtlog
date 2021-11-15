cmake_minimum_required(VERSION 3.15)

project(fmtlog CXX)

if(MSVC)
    add_compile_options(/std:c++latest)
else()
    add_compile_options(-Wall -O3 -std=c++17)
    link_libraries(pthread)
endif()

link_directories(.)
include_directories(fmt/include)

add_library(fmtlog-shared SHARED fmtlog.cc)
if(MSVC)
  target_link_libraries(fmtlog-shared fmt)
endif()
install(TARGETS fmtlog-shared)

add_library(fmtlog-static fmtlog.cc)
if(MSVC)
  target_link_libraries(fmtlog-static fmt)
endif()
install(TARGETS fmtlog-static)

add_subdirectory(fmt)
add_subdirectory(test)
if(NOT MSVC)
  add_subdirectory(bench)
endif()
