cmake_minimum_required(VERSION 3.15)

project(
  fmtlog
  LANGUAGES CXX
  VERSION 2.2.2)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

option(FMTLOG_USE_SYSTEM_FMT
       "Use system fmt library instead of bundled one" OFF)

if(FMTLOG_USE_SYSTEM_FMT)
  find_package(fmt CONFIG REQUIRED)
else()
  add_subdirectory(fmt)
endif()
find_package(Threads MODULE REQUIRED)

# =================================================================================================

add_library(${PROJECT_NAME}_flags INTERFACE)
target_compile_features(${PROJECT_NAME}_flags INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME}_flags INTERFACE Threads::Threads)

if(NOT MSVC)
  target_compile_options(${PROJECT_NAME}_flags INTERFACE -Wall $<$<CONFIG:Debug>:-O3>)
endif()

# =================================================================================================

add_library(${PROJECT_NAME}-shared SHARED fmtlog.cc)
target_include_directories(${PROJECT_NAME}-shared
                           INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(
  ${PROJECT_NAME}-shared
  PUBLIC fmt::fmt
  PRIVATE ${PROJECT_NAME}_flags)

add_library(${PROJECT_NAME}-static STATIC fmtlog.cc)
target_include_directories(${PROJECT_NAME}-static
                           INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(
  ${PROJECT_NAME}-static
  PUBLIC fmt::fmt
  PRIVATE ${PROJECT_NAME}_flags)

# =================================================================================================

set(fmtlog_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/fmtlog")

install(
  TARGETS ${PROJECT_NAME}-shared ${PROJECT_NAME}-static ${PROJECT_NAME}_flags
  EXPORT fmtlog_target
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES ${CMAKE_SOURCE_DIR}/fmtlog.h ${CMAKE_SOURCE_DIR}/fmtlog-inl.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/fmtlog/)

install(EXPORT fmtlog_target NAMESPACE fmtlog:: FILE fmtlogTargets.cmake
        DESTINATION ${fmtlog_INSTALL_CMAKEDIR})

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/fmtlogConfig.cmake.in ${PROJECT_BINARY_DIR}/fmtlogConfig.cmake
  INSTALL_DESTINATION ${fmtlog_INSTALL_CMAKEDIR})

write_basic_package_version_file(${PROJECT_BINARY_DIR}/fmtlogConfigVersion.cmake
                                 COMPATIBILITY SameMajorVersion)

install(FILES ${PROJECT_BINARY_DIR}/fmtlogConfig.cmake
              ${PROJECT_BINARY_DIR}/fmtlogConfigVersion.cmake
        DESTINATION ${fmtlog_INSTALL_CMAKEDIR})

# =================================================================================================

option(BUILD_TESTING "Build tests" ON)

if(BUILD_TESTING)
  include(CTest)
  enable_testing()
  add_subdirectory(test)
endif()
