function(set_project_lto_opts project_name)
  # if(${CMAKE_BUILD_TYPE} STREQUAL "Debug") message(AUTHOR_WARNING "The project ${PROJECT_NAME} is
  # building in debug mode, thus no compiler flags for link time optimization were added.") return()
  # endif()

  set(MSVC_LTO # sorry idk
  )

  if(${PROJECT_NAME}_CLANG_LTO_THIN)
    set(CLANG_LTO -flto=thin -Werror=odr -Werror=strict-aliasing)
  else()
    set(CLANG_LTO -flto -Werror=odr -Werror=strict-aliasing)
  endif()

  set(GCC_LTO -flto -Werror=odr -Werror=lto-type-mismatch -Werror=strict-aliasing)

  # if(MSVC) set(PROJECT_LTO ${MSVC_LTO}) elseif
  if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(PROJECT_LTO ${CLANG_LTO})
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(PROJECT_LTO ${GCC_LTO})
  else()
    message(
      AUTHOR_WARNING
        "No compiler flags for link time optimization were set for '${CMAKE_CXX_COMPILER_ID}' compiler."
    )
  endif()

  if(${PROJECT_NAME}_BUILD_HEADERS_ONLY)
    target_compile_options(${project_name} INTERFACE ${PROJECT_LTO})
    target_link_options(${project_name} INTERFACE ${PROJECT_LTO})
  else()
    target_compile_options(${project_name} PUBLIC ${PROJECT_LTO})
    target_link_options(${project_name} PUBLIC ${PROJECT_LTO})
  endif()

  if(NOT TARGET ${project_name})
    message(
      AUTHOR_WARNING
        "${project_name} is not a target, thus no compiler flags for link time optimization were added."
    )
  endif()
endfunction()
