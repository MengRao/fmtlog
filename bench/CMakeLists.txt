add_library(NanoLog NanoLog/runtime/Cycles.cc NanoLog/runtime/Util.cc NanoLog/runtime/Log.cc NanoLog/runtime/NanoLog.cc NanoLog/runtime/RuntimeLogger.cc NanoLog/runtime/TimeTrace.cc)

add_executable(bench bench.cc)
target_include_directories(bench PRIVATE spdlog/include)
target_compile_options(bench PRIVATE -Wno-unused-function)
target_link_libraries(bench fmtlog-static fmt NanoLog rt)
install(TARGETS bench)
