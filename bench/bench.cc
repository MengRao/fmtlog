#include <bits/stdc++.h>
#include <chrono>

#include "../fmtlog.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/async.h"
#include "NanoLog/runtime/NanoLogCpp17.h"

namespace GeneratedFunctions {
size_t numLogIds;
};

struct FmtLogBase
{
  void flush() { fmtlog::poll(true); }
};

struct StaticString : public FmtLogBase
{
  inline void log() { logi("Starting backup replica garbage collector thread"); }
};

struct StringConcat : public FmtLogBase
{
  inline void log() { logi("Opened session with coordinator at {}", "basic+udp:host=192.168.1.140,port=12246"); }
};

struct SingleInteger : public FmtLogBase
{
  inline void log() { logi("Backup storage speeds (min): {} MB/s read", 181); }
};

struct TwoIntegers : public FmtLogBase
{
  inline void log() {
    logi("buffer has consumed {} bytes of extra storage, current allocation: {} bytes", 1032024, 1016544);
  }
};

struct SingleDouble : public FmtLogBase
{
  inline void log() { logi("Using tombstone ratio balancer with ratio = {}", 0.400000); }
};

struct ComplexFormat : public FmtLogBase
{
  inline void log() {
    logi("Initialized InfUdDriver buffers: {} receive buffers ({} MB), {} transmit buffers ({} MB), took {:.1f} ms",
         50000, 97, 50, 0, 26.2);
  }
};

struct NanoLogBase
{
  void flush() { NanoLog::sync(); }
};

struct NanoLogStaticString : public NanoLogBase
{
  inline void log() { NANO_LOG(NOTICE, "Starting backup replica garbage collector thread"); }
};

struct NanoLogStringConcat : public NanoLogBase
{
  inline void log() {
    NANO_LOG(NOTICE, "Opened session with coordinator at %s", "basic+udp:host=192.168.1.140,port=12246");
  }
};

struct NanoLogSingleInteger : public NanoLogBase
{
  inline void log() { NANO_LOG(NOTICE, "Backup storage speeds (min): %d MB/s read", 181); }
};

struct NanoLogTwoIntegers : public NanoLogBase
{
  inline void log() {
    NANO_LOG(NOTICE, "buffer has consumed %u bytes of extra storage, current allocation: %u bytes", 1032024, 1016544);
  }
};

struct NanoLogSingleDouble : public NanoLogBase
{
  inline void log() { NANO_LOG(NOTICE, "Using tombstone ratio balancer with ratio = %0.6lf", 0.400000); }
};

struct NanoLogComplexFormat : public NanoLogBase
{
  inline void log() {
    NANO_LOG(NOTICE,
             "Initialized InfUdDriver buffers: %u receive buffers (%u MB), %u transmit buffers (%u MB), took %0.1lf ms",
             50000, 97, 50, 0, 26.2);
  }
};

struct SpdlogBase
{
  SpdlogBase(spdlog::logger* logger)
    : logger(logger) {}

  void flush() { logger->flush(); }

  spdlog::logger* logger;
};

struct SpdlogStaticString : public SpdlogBase
{
  SpdlogStaticString(spdlog::logger* logger)
    : SpdlogBase(logger) {}
  inline void log() { SPDLOG_LOGGER_INFO(logger, "Starting backup replica garbage collector thread"); }
};

struct SpdlogStringConcat : public SpdlogBase
{
  SpdlogStringConcat(spdlog::logger* logger)
    : SpdlogBase(logger) {}
  inline void log() {
    SPDLOG_LOGGER_INFO(logger, "Opened session with coordinator at {}", "basic+udp:host=192.168.1.140,port=12246");
  }
};

struct SpdlogSingleInteger : public SpdlogBase
{
  SpdlogSingleInteger(spdlog::logger* logger)
    : SpdlogBase(logger) {}
  inline void log() { SPDLOG_LOGGER_INFO(logger, "Backup storage speeds (min): {} MB/s read", 181); }
};

struct SpdlogTwoIntegers : public SpdlogBase
{
  SpdlogTwoIntegers(spdlog::logger* logger)
    : SpdlogBase(logger) {}
  inline void log() {
    SPDLOG_LOGGER_INFO(logger, "buffer has consumed {} bytes of extra storage, current allocation: {} bytes", 1032024,
                       1016544);
  }
};

struct SpdlogSingleDouble : public SpdlogBase
{
  SpdlogSingleDouble(spdlog::logger* logger)
    : SpdlogBase(logger) {}
  inline void log() { SPDLOG_LOGGER_INFO(logger, "Using tombstone ratio balancer with ratio = {}", 0.400000); }
};

struct SpdlogComplexFormat : public SpdlogBase
{
  SpdlogComplexFormat(spdlog::logger* logger)
    : SpdlogBase(logger) {}
  inline void log() {
    SPDLOG_LOGGER_INFO(
      logger,
      "Initialized InfUdDriver buffers: {} receive buffers ({} MB), {} transmit buffers ({} MB), took {:.1f} ms", 50000,
      97, 50, 0, 26.2);
  }
};

template<typename T>
void bench(T o) {
  const int RECORDS = 10000;
  std::chrono::high_resolution_clock::time_point t0, t1, t2;
  t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < RECORDS; ++i) {
    o.log();
  }
  t1 = std::chrono::high_resolution_clock::now();
  o.flush();
  t2 = std::chrono::high_resolution_clock::now();
  double span1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
  double span2 = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0).count();
  fmt::print("{}: front-end latency is {:.1f} ns/msg average, throughput  is {:.2f} million msgs/sec average\n",
             typeid(o).name(), (span1 / RECORDS) * 1e9, RECORDS / span2 / 1e6);
}

int main() {
  fmtlog::setLogFile("fmtlog.txt", true);
  fmtlog::setHeaderPattern("[{YmdHMSe}] [fmtlog] [{l}] [{s}] ");
  fmtlog::preallocate();

  NanoLog::setLogFile("nanalog.bin");
  NanoLog::preallocate();

  auto spdlogger = spdlog::basic_logger_st("spdlog", "spdlog.txt", true);

  bench(StaticString());
  bench(StringConcat());
  bench(SingleInteger());
  bench(TwoIntegers());
  bench(SingleDouble());
  bench(ComplexFormat());

  fmt::print("\n");

  bench(NanoLogStaticString());
  bench(NanoLogStringConcat());
  bench(NanoLogSingleInteger());
  bench(NanoLogTwoIntegers());
  bench(NanoLogSingleDouble());
  bench(NanoLogComplexFormat());

  fmt::print("\n");

  bench(SpdlogStaticString(spdlogger.get()));
  bench(SpdlogStringConcat(spdlogger.get()));
  bench(SpdlogSingleInteger(spdlogger.get()));
  bench(SpdlogTwoIntegers(spdlogger.get()));
  bench(SpdlogSingleDouble(spdlogger.get()));
  bench(SpdlogComplexFormat(spdlogger.get()));

  return 0;
}
