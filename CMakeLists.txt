cmake_minimum_required(VERSION 3.14...3.22)

# ---- Project ----

# Note: update this to your new project's name and version
set(fmtlog_VERSION 2.3.0)
project(
  fmtlog
  VERSION ${fmtlog_VERSION}
  LANGUAGES CXX)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
  set(default_value_options ON)
else()
  set(default_value_options OFF)
endif()

option(${PROJECT_NAME}_BUILD_WITH_ASAN
       "add -fsanitize=address aka turn on ASAN?" ${default_value_options})
option(BUILD_SHARED_LIBS "yes/no" NO)

if(${BUILD_SHARED_LIBS})
  # https://stackoverflow.com/questions/33062728/cmake-link-shared-library-on-windows
  if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
  endif()
endif()

set(FETCHCONTENT_QUIET
    OFF
    CACHE BOOL "Make downloading of packages quiet")

# ---- Include guards ----

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
  )
endif()

# ---- Add source files ----

# Note: globbing sources is considered bad practice as CMake's generators may
# not detect new files automatically. Keep that in mind when changing files, or
# explicitly mention them here.
file(GLOB_RECURSE headers CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")
# file(GLOB_RECURSE sources CONFIGURE_DEPENDS
# "${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp")

# ---- Create library ----

# Note: for header-only libraries change all PUBLIC flags to INTERFACE and
# create an interface target:
add_library(${PROJECT_NAME} INTERFACE)
# add_library(${PROJECT_NAME} ${headers} ${sources})
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

target_include_directories(
  ${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# being a cross-platform target, we enforce standards conformance on MSVC
target_compile_options(
  ${PROJECT_NAME} INTERFACE "$<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/permissive->")

if(CMAKE_BUILD_TYPE STREQUAL Debug)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${PROJECT_NAME}
                           INTERFACE -fconcepts-diagnostics-depth=10)
  endif()
  if(${${PROJECT_NAME}_BUILD_WITH_ASAN})
    target_compile_options(${PROJECT_NAME} INTERFACE -fsanitize=address)
    target_link_options(${PROJECT_NAME} INTERFACE -fsanitize=address)
  endif()
endif()

# ---- Add dependencies via CPM ----
# see https://github.com/TheLartians/CPM.cmake for more info

include(cmake/getCPM.cmake)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/packages/add_fmt.cmake)
target_link_libraries(${PROJECT_NAME} INTERFACE fmt::fmt-header-only)
# ---- Create an installable target ----
# this allows users to install and find the library via `find_package()`.

include(${CMAKE_CURRENT_LIST_DIR}/cmake/packages/add_packageproject.cmake)

# the location where the project's version header will be placed should match
# the project's regular header paths
string(TOLOWER ${PROJECT_NAME}/version.h VERSION_HEADER_LOCATION)
string(TOLOWER ${PROJECT_NAME}/export.h EXPORT_HEADER_LOCATION)

set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${PROJECT_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${PROJECT_VERSION})

packageproject(
  NAME
  ${PROJECT_NAME}
  VERSION
  ${PROJECT_VERSION}
  NAMESPACE
  ${PROJECT_NAME}
  BINARY_DIR
  ${PROJECT_BINARY_DIR}
  INCLUDE_DIR
  ${PROJECT_SOURCE_DIR}/include
  INCLUDE_DESTINATION
  include/${PROJECT_NAME}
  VERSION_HEADER
  "${VERSION_HEADER_LOCATION}"
  EXPORT_HEADER
  "${EXPORT_HEADER_LOCATION}"
  COMPATIBILITY
  "AnyNewerVersion"
  DISABLE_VERSION_SUFFIX
  ON
  DEPENDENCIES
  "fmt ${fmt_VERSION}")
