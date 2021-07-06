/*
MIT License

Copyright (c) 2021 Meng Rao <raomeng1@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "fmtlog.h"
#include <mutex>
#include <thread>
#include <limits>
#include <ios>

#ifdef _WIN32
#include <windows.h>
#include <processthreadsapi.h>
#else
#include <sys/syscall.h>
#include <unistd.h>
#endif

template<int ___ = 0>
class fmtlogDetailT
{
public:
  // https://github.com/MengRao/str
  template<size_t SIZE>
  class Str
  {
  public:
    static const int Size = SIZE;
    char s[SIZE];

    Str() {}
    Str(const char* p) { *this = *(const Str<SIZE>*)p; }

    char& operator[](int i) { return s[i]; }
    char operator[](int i) const { return s[i]; }

    template<typename T>
    void fromi(T num) {
      if constexpr (Size & 1) {
        s[Size - 1] = '0' + (num % 10);
        num /= 10;
      }
      switch (Size & -2) {
        case 18: *(uint16_t*)(s + 16) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
        case 16: *(uint16_t*)(s + 14) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
        case 14: *(uint16_t*)(s + 12) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
        case 12: *(uint16_t*)(s + 10) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
        case 10: *(uint16_t*)(s + 8) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
        case 8: *(uint16_t*)(s + 6) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
        case 6: *(uint16_t*)(s + 4) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
        case 4: *(uint16_t*)(s + 2) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
        case 2: *(uint16_t*)(s + 0) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
      }
    }

    static constexpr const char* digit_pairs = "00010203040506070809"
                                               "10111213141516171819"
                                               "20212223242526272829"
                                               "30313233343536373839"
                                               "40414243444546474849"
                                               "50515253545556575859"
                                               "60616263646566676869"
                                               "70717273747576777879"
                                               "80818283848586878889"
                                               "90919293949596979899";
  };

  fmtlogDetailT()
    : flushDelay(3000000000) {
    args.reserve(4096);
    args.resize(parttenArgSize);

    fmtlogWrapper<>::impl.init();
    resetDate();
    fmtlog::setLogFile(stdout);
    setHeaderPattern("{HMSf} {s:<16} {l}[{t:<6}] ");
    logInfos.reserve(32);
    bgLogInfos.reserve(128);
    bgLogInfos.emplace_back(nullptr, nullptr, fmtlog::DBG, fmt::string_view());
    bgLogInfos.emplace_back(nullptr, nullptr, fmtlog::INF, fmt::string_view());
    bgLogInfos.emplace_back(nullptr, nullptr, fmtlog::WRN, fmt::string_view());
    bgLogInfos.emplace_back(nullptr, nullptr, fmtlog::ERR, fmt::string_view());
    threadBuffers.reserve(8);
    bgThreadBuffers.reserve(8);
    memset(membuf.data(), 0, membuf.capacity());
  }

  ~fmtlogDetailT() {
    stopPollingThread();
    poll(true);
    closeLogFile();
  }

  void setHeaderPattern(const char* pattern) {
    if (shouldDeallocateHeader) delete[] headerPattern.data();
    using namespace fmt::literals;
    for (int i = 0; i < parttenArgSize; i++) {
      reorderIdx[i] = parttenArgSize - 1;
    }
    headerPattern = fmtlog::unNameFormat<true>(
      pattern, reorderIdx, "a"_a = "", "b"_a = "", "C"_a = "", "Y"_a = "", "m"_a = "", "d"_a = "",
      "t"_a = "thread name", "F"_a = "", "f"_a = "", "e"_a = "", "S"_a = "", "M"_a = "", "H"_a = "",
      "l"_a = fmtlog::LogLevel(), "s"_a = "fmtlog.cc:123", "g"_a = "/home/raomeng/fmtlog/fmtlog.cc:123", "Ymd"_a = "",
      "HMS"_a = "", "HMSe"_a = "", "HMSf"_a = "", "HMSF"_a = "", "YmdHMS"_a = "", "YmdHMSe"_a = "", "YmdHMSf"_a = "",
      "YmdHMSF"_a = "");
    shouldDeallocateHeader = headerPattern.data() != pattern;

    setArg<0>(fmt::string_view(weekdayName.s, 3));
    setArg<1>(fmt::string_view(monthName.s, 3));
    setArg<2>(fmt::string_view(&year[2], 2));
    setArg<3>(fmt::string_view(year.s, 4));
    setArg<4>(fmt::string_view(month.s, 2));
    setArg<5>(fmt::string_view(day.s, 2));
    setArg<6>(fmt::string_view());
    setArg<7>(fmt::string_view(nanosecond.s, 9));
    setArg<8>(fmt::string_view(nanosecond.s, 6));
    setArg<9>(fmt::string_view(nanosecond.s, 3));
    setArg<10>(fmt::string_view(second.s, 2));
    setArg<11>(fmt::string_view(minute.s, 2));
    setArg<12>(fmt::string_view(hour.s, 2));
    setArg<13>(fmt::string_view(logLevel.s, 3));
    setArg<14>(fmt::string_view());
    setArg<15>(fmt::string_view());
    setArg<16>(fmt::string_view(year.s, 10)); // Ymd
    setArg<17>(fmt::string_view(hour.s, 8));  // HMS
    setArg<18>(fmt::string_view(hour.s, 12)); // HMSe
    setArg<19>(fmt::string_view(hour.s, 15)); // HMSf
    setArg<20>(fmt::string_view(hour.s, 18)); // HMSF
    setArg<21>(fmt::string_view(year.s, 19));   // YmdHMS
    setArg<22>(fmt::string_view(year.s, 23));   // YmdHMSe
    setArg<23>(fmt::string_view(year.s, 26));   // YmdHMSf
    setArg<24>(fmt::string_view(year.s, 29));   // YmdHMSF
  }

  class ThreadBufferDestroyer
  {
  public:
    explicit ThreadBufferDestroyer() {}

    void threadBufferCreated() {}

    ~ThreadBufferDestroyer() {
      if (fmtlog::threadBuffer != nullptr) {
        fmtlog::threadBuffer->shouldDeallocate = true;
        fmtlog::threadBuffer = nullptr;
      }
    }
  };

  struct StaticLogInfo
  {
    // Constructor
    constexpr StaticLogInfo(fmtlog::FormatToFn fn, const char* loc, fmtlog::LogLevel level, fmt::string_view fmtString)
      : formatToFn(fn)
      , formatString(fmtString)
      , location(loc)
      , logLevel(level)
      , argIdx(-1) {}

    void processLocation() {
      size_t size = strlen(location);
      const char* p = location + size;
      if (size > 255) {
        location = p - 255;
      }
      endPos = p - location;
      const char* base = location;
      while (p > location) {
        char c = *--p;
        if (c == '/' || c == '\\') {
          base = p + 1;
          break;
        }
      }
      basePos = base - location;
    }

    inline fmt::string_view getBase() { return fmt::string_view(location + basePos, endPos - basePos); }

    inline fmt::string_view getLocation() { return fmt::string_view(location, endPos); }

    fmtlog::FormatToFn formatToFn;
    fmt::string_view formatString;
    const char* location;
    uint8_t basePos;
    uint8_t endPos;
    fmtlog::LogLevel logLevel;
    int argIdx;
  };

  static thread_local ThreadBufferDestroyer sbc;
  int64_t midnightNs;
  fmt::string_view headerPattern;
  bool shouldDeallocateHeader = false;
  FILE* outputFp = nullptr;
  bool manageFp = false;
  int64_t flushDelay;
  int64_t nextFlushTime = (std::numeric_limits<int64_t>::max)();
  uint32_t flushBufSize = 8 * 1024;
  fmtlog::LogLevel flushLogLevel = fmtlog::OFF;
  std::mutex bufferMutex;
  std::vector<fmtlog::ThreadBuffer*> threadBuffers;
  std::vector<fmtlog::ThreadBuffer*> bgThreadBuffers;
  std::mutex logInfoMutex;
  std::vector<StaticLogInfo> logInfos;
  std::vector<StaticLogInfo> bgLogInfos;

  fmtlog::LogCBFn logCB = nullptr;
  fmtlog::LogLevel minCBLogLevel;

  fmtlog::MemoryBuffer membuf;

  const static int parttenArgSize = 25;
  uint32_t reorderIdx[parttenArgSize];
  Str<3> weekdayName;
  Str<3> monthName;
  Str<4> year;
  char dash1 = '-';
  Str<2> month;
  char dash2 = '-';
  Str<2> day;
  char space = ' ';
  Str<2> hour;
  char colon1 = ':';
  Str<2> minute;
  char colon2 = ':';
  Str<2> second;
  char dot1 = '.';
  Str<9> nanosecond;
  Str<3> logLevel;
  std::vector<fmt::basic_format_arg<fmtlog::Context>> args;

  volatile bool threadRunning = false;
  std::thread thr;

  void resetDate() {
    time_t rawtime = fmtlogWrapper<>::impl.tscns.rdns() / 1000000000;
    struct tm* timeinfo = localtime(&rawtime);
    timeinfo->tm_sec = timeinfo->tm_min = timeinfo->tm_hour = 0;
    midnightNs = mktime(timeinfo) * 1000000000;
    year.fromi(1900 + timeinfo->tm_year);
    month.fromi(1 + timeinfo->tm_mon);
    day.fromi(timeinfo->tm_mday);
    const char* weekdays[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    weekdayName = weekdays[timeinfo->tm_wday];
    const char* monthNames[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    monthName = monthNames[timeinfo->tm_mon];
  }

  void preallocate() {
    if (fmtlog::threadBuffer) return;
    fmtlog::threadBuffer = new fmtlog::ThreadBuffer();
#ifdef _WIN32
    uint32_t tid = static_cast<uint32_t>(::GetCurrentThreadId());
#else
    uint32_t tid = static_cast<uint32_t>(::syscall(SYS_gettid));
#endif
    fmtlog::threadBuffer->nameSize =
      fmt::format_to_n(fmtlog::threadBuffer->name, sizeof(fmtlog::threadBuffer->name), "{}", tid).size;
    sbc.threadBufferCreated();

    std::unique_lock<std::mutex> guard(bufferMutex);
    threadBuffers.push_back(fmtlog::threadBuffer);
  }

  template<size_t I, typename T>
  inline void setArg(const T& arg) {
    args[reorderIdx[I]] = fmt::detail::make_arg<fmtlog::Context>(arg);
  }

  template<size_t I, typename T>
  inline void setArgVal(const T& arg) {
    fmt::detail::value<fmtlog::Context>& value_ = *(fmt::detail::value<fmtlog::Context>*)&args[reorderIdx[I]];
    value_ = fmt::detail::arg_mapper<fmtlog::Context>().map(arg);
  }

  void flushLogFile() {
    if (outputFp) {
      fwrite(membuf.data(), 1, membuf.size(), outputFp);
      if (!manageFp) fflush(outputFp);
    }
    membuf.clear();
    nextFlushTime = (std::numeric_limits<int64_t>::max)();
  }

  void closeLogFile() {
    if (manageFp) fclose(outputFp);
    outputFp = nullptr;
    manageFp = false;
  }

  void startPollingThread(int64_t pollInterval) {
    stopPollingThread();
    threadRunning = true;
    thr = std::thread([pollInterval, this]() {
      while (threadRunning) {
        int64_t before = fmtlogWrapper<>::impl.tscns.rdns();
        poll(false);
        int64_t delay = fmtlogWrapper<>::impl.tscns.rdns() - before;
        if (delay < pollInterval) {
          std::this_thread::sleep_for(std::chrono::nanoseconds(pollInterval - delay));
        }
      }
      poll(true);
    });
  }

  void stopPollingThread() {
    if (!threadRunning) return;
    threadRunning = false;
    if (thr.joinable()) thr.join();
  }

  void poll(bool forceFlush) {
    if (logInfos.size()) {
      std::unique_lock<std::mutex> lock(logInfoMutex);
      for (auto& info : logInfos) {
        info.processLocation();
      }
      bgLogInfos.insert(bgLogInfos.end(), logInfos.begin(), logInfos.end());
      logInfos.clear();
    }
    if (threadBuffers.size()) {
      std::unique_lock<std::mutex> lock(bufferMutex);
      bgThreadBuffers.insert(bgThreadBuffers.end(), threadBuffers.begin(), threadBuffers.end());
      threadBuffers.clear();
    }

    for (size_t i = 0; i < bgThreadBuffers.size(); i++) {
      fmtlog::ThreadBuffer* tb = bgThreadBuffers[i];
      setArgVal<6>(fmt::string_view(tb->name, tb->nameSize));
      while (true) {
        auto header = tb->varq.front();
        if (!header) {
          if (tb->shouldDeallocate) {
            delete tb;

            std::swap(bgThreadBuffers[i], bgThreadBuffers.back());
            bgThreadBuffers.pop_back();
            i--;
          }
          break;
        }

        if (header->logId >= bgLogInfos.size()) break; // it's just put into logInfos, handle it in the next poll
        StaticLogInfo& info = bgLogInfos[header->logId];
        char* end = (char*)header + header->size;
        char* data = (char*)(header + 1);
        if (!info.formatToFn) { // log once
          info.location = *(const char**)data;
          data += 8;
          info.processLocation();
        }
        int64_t tsc = *(int64_t*)data;
        data += 8;
        int64_t ts = fmtlogWrapper<>::impl.tscns.tsc2ns(tsc);
        // the date could go back when polling different threads
        uint64_t t = (ts > midnightNs) ? (ts - midnightNs) : 0;
        nanosecond.fromi(t % 1000000000);
        t /= 1000000000;
        second.fromi(t % 60);
        t /= 60;
        minute.fromi(t % 60);
        t /= 60;
        uint32_t h = t; // hour
        if (h > 23) {
          h %= 24;
          resetDate();
        }
        hour.fromi(h);
        setArgVal<14>(info.getBase());
        setArgVal<15>(info.getLocation());
        logLevel = "DBG INF WRN ERR OFF" + (info.logLevel << 2);

        size_t headerPos = membuf.size();
        fmt::detail::vformat_to(membuf, headerPattern, fmt::basic_format_args(args.data(), parttenArgSize));
        size_t bodyPos = membuf.size();

        if (info.formatToFn) {
          info.formatToFn(info.formatString, data, membuf, info.argIdx, args);
        }
        else { // log once
          membuf.append(fmt::string_view(data, end - data));
        }
        tb->varq.pop();

        if (logCB && info.logLevel >= minCBLogLevel) {
          logCB(ts, info.logLevel, info.getLocation(), info.basePos, fmt::string_view(tb->name, tb->nameSize),
                fmt::string_view(membuf.data() + headerPos, membuf.size() - headerPos), bodyPos - headerPos);
        }
        membuf.push_back('\n');
        if (membuf.size() >= flushBufSize || info.logLevel >= flushLogLevel) {
          flushLogFile();
        }
      }
    }
    if (membuf.size() == 0) return;
    if (!manageFp || forceFlush) {
      flushLogFile();
      return;
    }
    int64_t now = fmtlogWrapper<>::impl.tscns.rdns();
    if (now > nextFlushTime) {
      flushLogFile();
    }
    else if (nextFlushTime == (std::numeric_limits<int64_t>::max)()) {
      nextFlushTime = now + flushDelay;
    }
  }
};

template<int _>
thread_local typename fmtlogDetailT<_>::ThreadBufferDestroyer fmtlogDetailT<_>::sbc;

template<int __ = 0>
struct fmtlogDetailWrapper
{ static fmtlogDetailT<> impl; };

template<int _>
fmtlogDetailT<> fmtlogDetailWrapper<_>::impl;

template<int _>
void fmtlogT<_>::registerLogInfo(uint32_t& logId, FormatToFn fn, const char* location, LogLevel level,
                                 fmt::string_view fmtString) {
  auto& d = fmtlogDetailWrapper<>::impl;
  std::lock_guard<std::mutex> lock(d.logInfoMutex);
  if (logId) return;
  logId = d.logInfos.size() + d.bgLogInfos.size();
  d.logInfos.emplace_back(fn, location, level, fmtString);
}

template<int _>
void fmtlogT<_>::preallocate() {
  fmtlogDetailWrapper<>::impl.preallocate();
}

template<int _>
void fmtlogT<_>::setLogFile(const char* filename, bool truncate) {
  auto& d = fmtlogDetailWrapper<>::impl;
  FILE* newFp = fopen(filename, truncate ? "w" : "a");
  if (!newFp) {
    std::string err = fmt::format("Unable to open file: {}: {}", filename, strerror(errno));
    throw std::ios_base::failure(err);
  }
  setbuf(newFp, nullptr);

  closeLogFile();
  d.outputFp = newFp;
  d.manageFp = true;
}

template<int _>
void fmtlogT<_>::setLogFile(FILE* fp, bool manageFp) {
  auto& d = fmtlogDetailWrapper<>::impl;
  closeLogFile();
  if (manageFp) {
    setbuf(fp, nullptr);
  }
  d.outputFp = fp;
  d.manageFp = manageFp;
}

template<int _>
void fmtlogT<_>::setFlushDelay(int64_t ns) {
  fmtlogDetailWrapper<>::impl.flushDelay = ns;
}

template<int _>
void fmtlogT<_>::flushOn(LogLevel flushLogLevel) {
  fmtlogDetailWrapper<>::impl.flushLogLevel = flushLogLevel;
}

template<int _>
void fmtlogT<_>::setFlushBufSize(uint32_t bytes) {
  fmtlogDetailWrapper<>::impl.flushBufSize = bytes;
}

template<int _>
void fmtlogT<_>::closeLogFile() {
  fmtlogDetailWrapper<>::impl.closeLogFile();
}

template<int _>
void fmtlogT<_>::poll(bool forceFlush) {
  fmtlogDetailWrapper<>::impl.poll(forceFlush);
}

template<int _>
void fmtlogT<_>::setThreadName(const char* name) {
  preallocate();
  threadBuffer->nameSize = fmt::format_to_n(threadBuffer->name, sizeof(fmtlog::threadBuffer->name), "{}", name).size;
}

template<int _>
void fmtlogT<_>::setLogCB(LogCBFn cb, LogLevel minCBLogLevel_) {
  auto& d = fmtlogDetailWrapper<>::impl;
  d.logCB = cb;
  d.minCBLogLevel = minCBLogLevel_;
}

template<int _>
void fmtlogT<_>::setHeaderPattern(const char* pattern) {
  fmtlogDetailWrapper<>::impl.setHeaderPattern(pattern);
}

template<int _>
void fmtlogT<_>::startPollingThread(int64_t pollInterval) {
  fmtlogDetailWrapper<>::impl.startPollingThread(pollInterval);
}

template<int _>
void fmtlogT<_>::stopPollingThread() {
  fmtlogDetailWrapper<>::impl.stopPollingThread();
}

template<int _>
void fmtlogT<_>::setTscGhz(double tscGhz) {
  fmtlogWrapper<>::impl.tscns.init(tscGhz);
}

template class fmtlogT<0>;

