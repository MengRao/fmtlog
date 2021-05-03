#include <bits/stdc++.h>
#include <chrono>
#include <unistd.h>

#define FMTLOG_ACTIVE_LEVEL FMTLOG_LEVEL_DBG
#include "../fmtlog.h"

void runBenchmark();

void logcb(uint64_t ns, fmtlog::LogLevel level, fmt::string_view location, size_t basePos, fmt::string_view threadName,
           fmt::string_view msg, size_t bodyPos) {
  fmt::print("callback full msg: {}\n", msg);
  msg.remove_prefix(bodyPos);
  fmt::print("callback msg body: {}\n", msg);
}

int main() {
  char randomString[] = "Hello World";
  logi("A string, pointer, number, and float: '{}', {}, {}, {}", randomString, (void*)&randomString, 512, 3.14159);

  // logi("This msg will trigger compile error: {", 123);

  logd("This message wont be logged since it is lower "
       "than the current log level.");
  fmtlog::setLogLevel(fmtlog::DBG);
  logd("Now debug msg is shown");

  fmtlog::poll();

  fmtlog::setThreadName("main");
  logi("Thread name changed");

  fmtlog::poll();

  fmtlog::setHeaderPattern("{YmdHMSF} {s} {l}[{t}] ");
  logi("Header pattern is changed, full date time info is shown");

  fmtlog::poll();

  for (int i = 0; i < 10; i++) {
    logil(10, "This msg will be logged at an interval of at least 10 ns: {}.", i);
  }

  fmtlog::poll();

  fmtlog::setLogCB(logcb, fmtlog::WRN);
  logw("This msg will be called back");

  fmtlog::poll();
  runBenchmark();

  return 0;
}

void runBenchmark() {
  const int RECORDS = 10000;
  fmtlog::setLogFile("/dev/null", false);
  fmtlog::setLogCB(nullptr, fmtlog::WRN);

  std::chrono::high_resolution_clock::time_point t0, t1;

  t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < RECORDS; ++i) {
    logi("Simple log message with one parameters, {}", i);
  }
  t1 = std::chrono::high_resolution_clock::now();

  double span = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
  fmt::print("benchmark, front latency: {:.2} ns/msg average\n", (span / RECORDS) * 1e9);
}
