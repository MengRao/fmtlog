cmake_minimum_required(VERSION 3.15)
project(fmtlog_test LANGUAGES CXX)

if(NOT MSVC)
  add_library(static_lib lib.cc)

  add_library(static_header_lib lib.cc)
  target_compile_definitions(static_header_lib PUBLIC FMTLOG_HEADER_ONLY)

  add_library(shared_lib SHARED lib.cc)

  add_library(shared_header_lib SHARED lib.cc)
  target_compile_definitions(shared_header_lib PUBLIC FMTLOG_HEADER_ONLY)

  add_executable(link_static_static link_test.cc)
  target_link_libraries(link_static_static PRIVATE fmtlog-static static_lib)
  add_test(NAME link_static_static COMMAND link_static_static)

  add_executable(link_static_shared link_test.cc)
  target_link_libraries(link_static_shared PRIVATE fmtlog-static shared_lib)
  add_test(NAME link_static_shared COMMAND link_static_shared)

  add_executable(link_shared_static link_test.cc)
  target_link_libraries(link_shared_static PRIVATE fmtlog-shared static_lib)
  add_test(NAME link_shared_static COMMAND link_shared_static)

  add_executable(link_shared_shared link_test.cc)
  target_link_libraries(link_shared_shared PRIVATE fmtlog-shared shared_lib)
  add_test(NAME link_shared_shared COMMAND link_shared_shared)

  add_executable(link_header_static link_test.cc)
  target_link_libraries(link_header_static static_header_lib fmt)
  add_test(NAME link_header_static COMMAND link_header_static)

  add_executable(link_header_shared link_test.cc)
  target_link_libraries(link_header_shared shared_header_lib fmt)
  add_test(NAME link_header_shared COMMAND link_header_shared)
endif()

add_executable(log_test log_test.cc)
target_link_libraries(log_test PRIVATE fmtlog-static)
#target_compile_definitions(log_test PUBLIC FMTLOG_HEADER_ONLY)
add_test(NAME log_test COMMAND log_test)

add_executable(enc_dec_test enc_dec_test.cc)
target_link_libraries(enc_dec_test PRIVATE fmtlog-static)
add_test(NAME enc_dec_test COMMAND enc_dec_test)

add_executable(multithread_test multithread_test.cc)
target_compile_definitions(multithread_test PUBLIC FMTLOG_HEADER_ONLY=1)
target_link_libraries(multithread_test PRIVATE fmtlog-static)
add_test(NAME multithread_test COMMAND multithread_test)

