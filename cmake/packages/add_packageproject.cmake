include(${CMAKE_CURRENT_LIST_DIR}/../getCPM.cmake)

# PackageProject.cmake will be used to make our target installable
file(GLOB_RECURSE patches_for_boost CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/patches/packageProject.cmake/*.patch"
)

set(packageProject_VERSION 1.12.0)
CPMAddPackage(
  NAME PackageProject.cmake
  VERSION ${packageProject_VERSION}
  # GIT_REPOSITORY "https://github.com/TheLartians/PackageProject.cmake.git" GIT_TAG
  # "v${packageProject_VERSION}"
  URL "https://github.com/TheLartians/PackageProject.cmake/archive/refs/tags/v${packageProject_VERSION}.zip"
      PATCHES ${patches_for_boost}
)
