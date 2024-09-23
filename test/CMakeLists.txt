cmake_minimum_required(VERSION 3.14...3.22)

project(fmtlogTests LANGUAGES CXX)

# ---- Options ----

option(ENABLE_TEST_COVERAGE "Enable test coverage" OFF)
option(TEST_INSTALLED_VERSION "Test the version found by find_package" OFF)

# --- Import tools ----

include(../cmake/tools.cmake)

# ---- Dependencies ----

include(../cmake/getCPM.cmake)
CPMAddPackage("gh:TheLartians/Format.cmake@1.7.3")

if(TEST_INSTALLED_VERSION)
  find_package(fmtlog REQUIRED)
else()
  CPMAddPackage(NAME fmtlog SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
endif()

# ---- Create binary ----

file(
  GLOB_RECURSE sources CONFIGURE_DEPENDS
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
)

foreach(source ${sources})
  string(REPLACE "." "_" name ${source})
  string(REPLACE "/" "_" name ${source})
  message(DEBUG "Adding test with name ${PROJECT_NAME}_test_${name}")
  add_executable(${PROJECT_NAME}_test_${name} ${CMAKE_CURRENT_SOURCE_DIR}/src/${source})
  target_link_libraries(${PROJECT_NAME}_test_${name} PRIVATE fmtlog::fmtlog)
  set_target_properties(${PROJECT_NAME}_test_${name} PROPERTIES CXX_STANDARD 17)
  add_test(NAME ${PROJECT_NAME}_test_${name} COMMAND ${PROJECT_NAME}_test_${name})
endforeach()
# enable compiler warnings
if(NOT TEST_INSTALLED_VERSION)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(fmtlog INTERFACE -Wall -Wpedantic -Wextra)
  elseif(MSVC)
    target_compile_options(fmtlog INTERFACE /W4 /WX)
    # target_compile_definitions(${PROJECT_NAME} PUBLIC DOCTEST_CONFIG_USE_STD_HEADERS)
  endif()
endif()

# ---- Add fmtlogTests ----

enable_testing()

# Note: doctest and similar testing frameworks can automatically configure CMake tests. For other
# testing frameworks add the tests target instead: add_test(NAME ${PROJECT_NAME} COMMAND
# ${PROJECT_NAME})

# if(NOT DEFINED doctest_ADDED OR "${doctest_ADDED}" STREQUAL "")
# include("/usr/lib64/cmake/doctest/doctest.cmake") else()
# include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake) endif()
# doctest_discover_tests(${PROJECT_NAME})

# ---- code coverage ----

if(ENABLE_TEST_COVERAGE)
  target_compile_options(fmtlog INTERFACE -O0 -g -fprofile-arcs -ftest-coverage)
  target_link_options(fmtlog INTERFACE -fprofile-arcs -ftest-coverage)
endif()
