function(set_project_polyhedral_opts project_name)
  if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    message(
      AUTHOR_WARNING
        "The project ${PROJECT_NAME} is building in debug mode, thus no compiler flags for polyhedral optimization were added."
    )
    return()
  endif()

  set(MSVC_POLYHEDRAL # sorry idk
  )

  set(CLANG_POLLY -mllvm -polly)

  set(GCC_GRAPHITE -fgraphite-identity -floop-interchange -floop-strip-mine -floop-nest-optimize)

  # if(MSVC) set(PROJECT_POLYHEDRAL ${MSVC_POLYHEDRAL}) elseif
  if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(PROJECT_POLYHEDRAL ${CLANG_POLLY})
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(PROJECT_POLYHEDRAL ${GCC_GRAPHITE})
  else()
    message(
      AUTHOR_WARNING
        "No compiler flags for polyhedral optimization were set for '${CMAKE_CXX_COMPILER_ID}' compiler."
    )
  endif()

  if(${PROJECT_NAME}_BUILD_HEADERS_ONLY)
    target_compile_options(${project_name} INTERFACE ${PROJECT_POLYHEDRAL})
    target_link_options(${project_name} INTERFACE ${PROJECT_POLYHEDRAL})
  else()
    target_compile_options(${project_name} PUBLIC ${PROJECT_POLYHEDRAL})
    target_link_options(${project_name} PUBLIC ${PROJECT_POLYHEDRAL})
  endif()

  if(NOT TARGET ${project_name})
    message(
      AUTHOR_WARNING
        "${project_name} is not a target, thus no compiler flags for polyhedral optimization were added."
    )
  endif()
endfunction()
