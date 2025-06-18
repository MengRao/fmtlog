cmake_minimum_required(VERSION 3.15)

project(fmtlog CXX)

if(MSVC)
    add_compile_options(/std:c++latest)
else()
    add_compile_options(-Wall -O3 -std=c++17)
    #add_compile_options(-Wall -Ofast -std=c++2a -DNDEBUG -march=skylake -flto -fno-exceptions -fno-rtti -fno-unwind-tables -fno-asynchronous-unwind-tables -DFMTLOG_NO_CHECK_LEVEL=1)
    #SET(CMAKE_AR  "gcc-ar")
    #SET(CMAKE_RANLIB  "gcc-ranlib")
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
option(BUILD_TESTS_FMTLOG "Enable building tests" ON)
if(BUILD_TESTS_FMTLOG)
    add_subdirectory(test)
endif()