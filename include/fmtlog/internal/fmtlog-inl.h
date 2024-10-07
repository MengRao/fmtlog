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

#pragma once

#include <ios>
#include <limits>
#include <mutex>
#include <thread>

#include "fmtlog/fmtlog.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <processthreadsapi.h>
#else
#include <unistd.h>

#include <sys/syscall.h>
#endif

namespace
{
  void fmtlogEmptyFun(void*)
  {
  }
}  // namespace

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

    Str()
    {
    }
    Str(const char* p)
    {
      *this = *(const Str<SIZE>*)p;
    }

    char& operator[](int i)
    {
      return s[i];
    }
    char operator[](int i) const
    {
      return s[i];
    }

    template<typename T>
    void fromi(T num)
    {
      if constexpr (Size & 1)
      {
        s[Size - 1] = '0' + (num % 10);
        num /= 10;
      }
      switch (Size & -2)
      {
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

    static constexpr const char* digit_pairs =
        "00010203040506070809"
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

  fmtlogDetailT() : flushDelay(3000000000)
  {
    args.reserve(4096);
    args.resize(parttenArgSize);

    fmtlogWrapper<>::impl.init();
    resetDate();
    fmtlog::setLogFile(stdout);
    setHeaderPattern("{HMSf} {s:<16} {l}[{t:<6}] ");
    logInfos.reserve(32);
    bgLogInfos.reserve(128);
    bgLogInfos.emplace_back(nullptr, nullptr, fmtlog::DBG, std::string{});
    bgLogInfos.emplace_back(nullptr, nullptr, fmtlog::INF, std::string{});
    bgLogInfos.emplace_back(nullptr, nullptr, fmtlog::WRN, std::string{});
    bgLogInfos.emplace_back(nullptr, nullptr, fmtlog::ERR, std::string{});
    threadBuffers.reserve(8);
    bgThreadBuffers.reserve(8);
    memset(membuf.data(), 0, membuf.capacity());
  }

  ~fmtlogDetailT()
  {
    stopPollingThread();
    poll(true);
    closeLogFile();
  }

  void setHeaderPattern(std::string pattern)
  {
    // if (shouldDeallocateHeader) delete[] headerPattern.data();
    using namespace fmt::literals;
    for (int i = 0; i < parttenArgSize; i++)
    {
      reorderIdx[i] = parttenArgSize - 1;
    }
    headerPattern = fmtlog::unNameFormat<true>(
        pattern,
        reorderIdx,
        "a"_a = "",
        "b"_a = "",
        "C"_a = "",
        "Y"_a = "",
        "m"_a = "",
        "d"_a = "",
        "t"_a = "thread name",
        "F"_a = "",
        "f"_a = "",
        "e"_a = "",
        "S"_a = "",
        "M"_a = "",
        "H"_a = "",
        "l"_a = fmtlog::LogLevel(),
        "s"_a = "fmtlog.cc:123",
        "g"_a = "/home/raomeng/fmtlog/fmtlog.cc:123",
        "Ymd"_a = "",
        "HMS"_a = "",
        "HMSe"_a = "",
        "HMSf"_a = "",
        "HMSF"_a = "",
        "YmdHMS"_a = "",
        "YmdHMSe"_a = "",
        "YmdHMSf"_a = "",
        "YmdHMSF"_a = "");
    // shouldDeallocateHeader = headerPattern.data() != pattern;

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
    setArg<16>(fmt::string_view(year.s, 10));  // Ymd
    setArg<17>(fmt::string_view(hour.s, 8));   // HMS
    setArg<18>(fmt::string_view(hour.s, 12));  // HMSe
    setArg<19>(fmt::string_view(hour.s, 15));  // HMSf
    setArg<20>(fmt::string_view(hour.s, 18));  // HMSF
    setArg<21>(fmt::string_view(year.s, 19));  // YmdHMS
    setArg<22>(fmt::string_view(year.s, 23));  // YmdHMSe
    setArg<23>(fmt::string_view(year.s, 26));  // YmdHMSf
    setArg<24>(fmt::string_view(year.s, 29));  // YmdHMSF
  }

  class ThreadBufferDestroyer
  {
   public:
    explicit ThreadBufferDestroyer()
    {
    }

    void threadBufferCreated()
    {
    }

    ~ThreadBufferDestroyer()
    {
      if (fmtlog::threadBuffer != nullptr)
      {
        fmtlog::threadBuffer->shouldDeallocate = true;
        fmtlog::threadBuffer = nullptr;
      }
    }
  };

  struct StaticLogInfo
  {
    // Constructor
    StaticLogInfo(fmtlog::FormatToFn fn, const char* loc, fmtlog::LogLevel level, std::string fmtString)
        : formatToFn(fn),
          formatString(fmtString),
          location(loc),
          logLevel(level),
          argIdx(-1)
    {
    }

    void processLocation()
    {
      size_t size = strlen(location);
      const char* p = location + size;
      if (size > 255)
      {
        location = p - 255;
      }
      endPos = p - location;
      const char* base = location;
      while (p > location)
      {
        char c = *--p;
        if (c == '/' || c == '\\')
        {
          base = p + 1;
          break;
        }
      }
      basePos = base - location;
    }

    inline fmt::string_view getBase()
    {
      return fmt::string_view(location + basePos, endPos - basePos);
    }

    inline fmt::string_view getLocation()
    {
      return fmt::string_view(location, endPos);
    }

    fmtlog::FormatToFn formatToFn;
    std::string formatString;
    const char* location;
    uint8_t basePos;
    uint8_t endPos;
    fmtlog::LogLevel logLevel;
    int argIdx;
  };

  static thread_local ThreadBufferDestroyer sbc;
  int64_t midnightNs;
  std::string headerPattern;
  bool shouldDeallocateHeader = false;
  FILE* outputFp = nullptr;
  bool manageFp = false;
  size_t fpos = 0;  // file position of membuf, used only when manageFp == true
  int64_t flushDelay;
  int64_t nextFlushTime = (std::numeric_limits<int64_t>::max)();
  uint32_t flushBufSize = 8 * 1024;
  fmtlog::LogLevel flushLogLevel = fmtlog::OFF;
  std::mutex bufferMutex;
  std::vector<fmtlog::ThreadBuffer*> threadBuffers;
  struct HeapNode
  {
    HeapNode(fmtlog::ThreadBuffer* buffer) : tb(buffer)
    {
    }

    fmtlog::ThreadBuffer* tb;
    const fmtlog::SPSCVarQueueOPT::MsgHeader* header = nullptr;
  };
  std::vector<HeapNode> bgThreadBuffers;
  std::mutex logInfoMutex;
  std::vector<StaticLogInfo> logInfos;
  std::vector<StaticLogInfo> bgLogInfos;

  fmtlog::LogCBFn logCB = nullptr;
  fmtlog::LogLevel minCBLogLevel;
  fmtlog::LogQFullCBFn logQFullCB = fmtlogEmptyFun;
  void* logQFullCBArg = nullptr;

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

  void resetDate()
  {
    time_t rawtime = fmtlogWrapper<>::impl.tscns.rdns() / 1000000000;
    struct tm* timeinfo = localtime(&rawtime);
    timeinfo->tm_sec = timeinfo->tm_min = timeinfo->tm_hour = 0;
    midnightNs = mktime(timeinfo) * 1000000000;
    year.fromi(1900 + timeinfo->tm_year);
    month.fromi(1 + timeinfo->tm_mon);
    day.fromi(timeinfo->tm_mday);
    const char* weekdays[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    weekdayName = weekdays[timeinfo->tm_wday];
    const char* monthNames[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
    monthName = monthNames[timeinfo->tm_mon];
  }

  void preallocate()
  {
    if (fmtlog::threadBuffer)
      return;
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
  inline void setArg(const T& arg)
  {
    args[reorderIdx[I]] = fmt::detail::make_arg<fmtlog::Context>(arg);
  }

  template<size_t I, typename T>
  inline void setArgVal(const T& arg)
  {
    fmt::detail::value<fmtlog::Context>& value_ = *(fmt::detail::value<fmtlog::Context>*)&args[reorderIdx[I]];
    value_ = fmt::detail::arg_mapper<fmtlog::Context>().map(arg);
  }

  void flushLogFile()
  {
    if (outputFp)
    {
      fwrite(membuf.data(), 1, membuf.size(), outputFp);
      if (!manageFp)
        fflush(outputFp);
      else
        fpos += membuf.size();
    }
    membuf.clear();
    nextFlushTime = (std::numeric_limits<int64_t>::max)();
  }

  void closeLogFile()
  {
    if (membuf.size())
      flushLogFile();
    if (manageFp)
      fclose(outputFp);
    outputFp = nullptr;
    manageFp = false;
  }

  void startPollingThread(int64_t pollInterval)
  {
    stopPollingThread();
    threadRunning = true;
    thr = std::thread(
        [pollInterval, this]()
        {
          while (threadRunning)
          {
            int64_t before = fmtlogWrapper<>::impl.tscns.rdns();
            poll(false);
            int64_t delay = fmtlogWrapper<>::impl.tscns.rdns() - before;
            if (delay < pollInterval)
            {
              std::this_thread::sleep_for(std::chrono::nanoseconds(pollInterval - delay));
            }
          }
          poll(true);
        });
  }

  void stopPollingThread()
  {
    if (!threadRunning)
      return;
    threadRunning = false;
    if (thr.joinable())
      thr.join();
  }

  void handleLog(fmt::string_view threadName, const fmtlog::SPSCVarQueueOPT::MsgHeader* header)
  {
    setArgVal<6>(threadName);
    StaticLogInfo& info = bgLogInfos[header->logId];
    const char* data = (const char*)(header + 1);
    const char* end = (const char*)header + header->size;
    int64_t tsc = *(int64_t*)data;
    data += 8;
    if (!info.formatToFn)
    {  // log once
      info.location = *(const char**)data;
      data += 8;
      info.processLocation();
    }
    int64_t ts = fmtlogWrapper<>::impl.tscns.tsc2ns(tsc);
    // the date could go back when polling different threads
    uint64_t t = (ts > midnightNs) ? (ts - midnightNs) : 0;
    nanosecond.fromi(t % 1000000000);
    t /= 1000000000;
    second.fromi(t % 60);
    t /= 60;
    minute.fromi(t % 60);
    t /= 60;
    uint32_t h = t;  // hour
    if (h > 23)
    {
      h %= 24;
      resetDate();
    }
    hour.fromi(h);
    setArgVal<14>(info.getBase());
    setArgVal<15>(info.getLocation());
    logLevel = (const char*)"DBG INF WRN ERR OFF" + (info.logLevel << 2);

    size_t headerPos = membuf.size();
    fmtlog::vformat_to(membuf, headerPattern, fmt::basic_format_args(args.data(), parttenArgSize));
    size_t bodyPos = membuf.size();

    if (info.formatToFn)
    {
      info.formatToFn(info.formatString, data, membuf, info.argIdx, args);
    }
    else
    {  // log once
      membuf.append(fmt::string_view(data, end - data));
    }

    if (logCB && info.logLevel >= minCBLogLevel)
    {
      logCB(
          ts,
          info.logLevel,
          info.getLocation(),
          info.basePos,
          threadName,
          fmt::string_view(membuf.data() + headerPos, membuf.size() - headerPos),
          bodyPos - headerPos,
          fpos + headerPos);
    }
    membuf.push_back('\n');
    if (membuf.size() >= flushBufSize || info.logLevel >= flushLogLevel)
    {
      flushLogFile();
    }
  }

  void adjustHeap(size_t i)
  {
    while (true)
    {
      size_t min_i = i;
      for (size_t ch = i * 2 + 1, end = std::min(ch + 2, bgThreadBuffers.size()); ch < end; ch++)
      {
        auto h_ch = bgThreadBuffers[ch].header;
        auto h_min = bgThreadBuffers[min_i].header;
        if (h_ch && (!h_min || *(int64_t*)(h_ch + 1) < *(int64_t*)(h_min + 1)))
          min_i = ch;
      }
      if (min_i == i)
        break;
      std::swap(bgThreadBuffers[i], bgThreadBuffers[min_i]);
      i = min_i;
    }
  }

  void poll(bool forceFlush)
  {
    fmtlogWrapper<>::impl.tscns.calibrate();
    int64_t tsc = fmtlogWrapper<>::impl.tscns.rdtsc();
    if (logInfos.size())
    {
      std::unique_lock<std::mutex> lock(logInfoMutex);
      for (auto& info : logInfos)
      {
        info.processLocation();
      }
      bgLogInfos.insert(bgLogInfos.end(), logInfos.begin(), logInfos.end());
      logInfos.clear();
    }
    if (threadBuffers.size())
    {
      std::unique_lock<std::mutex> lock(bufferMutex);
      for (auto tb : threadBuffers)
      {
        bgThreadBuffers.emplace_back(tb);
      }
      threadBuffers.clear();
    }

    for (size_t i = 0; i < bgThreadBuffers.size(); i++)
    {
      auto& node = bgThreadBuffers[i];
      if (node.header)
        continue;
      node.header = node.tb->varq.front();
      if (node.tb->shouldDeallocate)
      {
        delete node.tb;
        node = bgThreadBuffers.back();
        bgThreadBuffers.pop_back();
        i--;
      }
    }

    if (bgThreadBuffers.empty())
      return;

    // build heap
    for (int i = bgThreadBuffers.size() / 2; i >= 0; i--)
    {
      adjustHeap(i);
    }

    while (true)
    {
      auto h = bgThreadBuffers[0].header;
      if (!h || h->logId >= bgLogInfos.size() || *(int64_t*)(h + 1) >= tsc)
        break;
      auto tb = bgThreadBuffers[0].tb;
      handleLog(fmt::string_view(tb->name, tb->nameSize), h);
      tb->varq.pop();
      bgThreadBuffers[0].header = tb->varq.front();
      adjustHeap(0);
    }

    if (membuf.size() == 0)
      return;
    if (!manageFp || forceFlush)
    {
      flushLogFile();
      return;
    }
    int64_t now = fmtlogWrapper<>::impl.tscns.tsc2ns(tsc);
    if (now > nextFlushTime)
    {
      flushLogFile();
    }
    else if (nextFlushTime == (std::numeric_limits<int64_t>::max)())
    {
      nextFlushTime = now + flushDelay;
    }
  }
};

template<int _>
thread_local typename fmtlogDetailT<_>::ThreadBufferDestroyer fmtlogDetailT<_>::sbc;

template<int __ = 0>
struct fmtlogDetailWrapper
{
  static fmtlogDetailT<> impl;
};

template<int _>
fmtlogDetailT<> fmtlogDetailWrapper<_>::impl;

template<int _>
void fmtlogT<_>::registerLogInfo(
    uint32_t& logId,
    FormatToFn fn,
    const char* location,
    LogLevel level,
    std::string fmtString) noexcept
{
  auto& d = fmtlogDetailWrapper<>::impl;
  std::lock_guard<std::mutex> lock(d.logInfoMutex);
  if (logId)
    return;
  logId = d.logInfos.size() + d.bgLogInfos.size();
  d.logInfos.emplace_back(fn, location, level, fmtString);
}

template<int _>
void fmtlogT<_>::vformat_to(fmtlog::MemoryBuffer& out, fmt::string_view fmt, fmt::format_args args)
{
  fmt::detail::vformat_to(out, fmt, args);
}

template<int _>
size_t fmtlogT<_>::formatted_size(fmt::string_view fmt, fmt::format_args args)
{
  auto buf = fmt::detail::counting_buffer<>();
  fmt::detail::vformat_to(buf, fmt, args);
  return buf.count();
}

template<int _>
void fmtlogT<_>::vformat_to(char* out, fmt::string_view fmt, fmt::format_args args)
{
  fmt::vformat_to(out, fmt, args);
}

template<int _>
typename fmtlogT<_>::SPSCVarQueueOPT::MsgHeader* fmtlogT<_>::allocMsg(uint32_t size, bool q_full_cb) noexcept
{
  auto& d = fmtlogDetailWrapper<>::impl;
  if (threadBuffer == nullptr)
    preallocate();
  auto ret = threadBuffer->varq.alloc(size);
  if ((ret == nullptr) & q_full_cb)
    d.logQFullCB(d.logQFullCBArg);
  return ret;
}

template<int _>
typename fmtlogT<_>::SPSCVarQueueOPT::MsgHeader* fmtlogT<_>::SPSCVarQueueOPT::allocMsg(uint32_t size) noexcept
{
  return alloc(size);
}

template<int _>
void fmtlogT<_>::preallocate() noexcept
{
  fmtlogDetailWrapper<>::impl.preallocate();
}

template<int _>
void fmtlogT<_>::setLogFile(const char* filename, bool truncate)
{
  auto& d = fmtlogDetailWrapper<>::impl;
  FILE* newFp = fopen(filename, truncate ? "w" : "a");
  if (!newFp)
  {
    std::string err = fmt::format("Unable to open file: {}: {}", filename, strerror(errno));
    fmt::throw_format_error(err.c_str());
  }
  setbuf(newFp, nullptr);
  d.fpos = ftell(newFp);

  closeLogFile();
  d.outputFp = newFp;
  d.manageFp = true;
}

template<int _>
void fmtlogT<_>::setLogFile(FILE* fp, bool manageFp)
{
  auto& d = fmtlogDetailWrapper<>::impl;
  closeLogFile();
  if (manageFp)
  {
    setbuf(fp, nullptr);
    d.fpos = ftell(fp);
  }
  else
    d.fpos = 0;
  d.outputFp = fp;
  d.manageFp = manageFp;
}

template<int _>
void fmtlogT<_>::setFlushDelay(int64_t ns) noexcept
{
  fmtlogDetailWrapper<>::impl.flushDelay = ns;
}

template<int _>
void fmtlogT<_>::flushOn(LogLevel flushLogLevel) noexcept
{
  fmtlogDetailWrapper<>::impl.flushLogLevel = flushLogLevel;
}

template<int _>
void fmtlogT<_>::setFlushBufSize(uint32_t bytes) noexcept
{
  fmtlogDetailWrapper<>::impl.flushBufSize = bytes;
}

template<int _>
void fmtlogT<_>::closeLogFile() noexcept
{
  fmtlogDetailWrapper<>::impl.closeLogFile();
}

template<int _>
void fmtlogT<_>::poll(bool forceFlush)
{
  fmtlogDetailWrapper<>::impl.poll(forceFlush);
}

template<int _>
void fmtlogT<_>::setThreadName(const char* name) noexcept
{
  preallocate();
  threadBuffer->nameSize = fmt::format_to_n(threadBuffer->name, sizeof(fmtlog::threadBuffer->name), "{}", name).size;
}

template<int _>
void fmtlogT<_>::setLogCB(LogCBFn cb, LogLevel minCBLogLevel_) noexcept
{
  auto& d = fmtlogDetailWrapper<>::impl;
  d.logCB = cb;
  d.minCBLogLevel = minCBLogLevel_;
}

template<int _>
void fmtlogT<_>::setLogQFullCB(LogQFullCBFn cb, void* userData) noexcept
{
  auto& d = fmtlogDetailWrapper<>::impl;
  d.logQFullCB = cb;
  d.logQFullCBArg = userData;
}

template<int _>
void fmtlogT<_>::setHeaderPattern(const char* pattern)
{
  fmtlogDetailWrapper<>::impl.setHeaderPattern(pattern);
}

template<int _>
void fmtlogT<_>::startPollingThread(int64_t pollInterval) noexcept
{
  fmtlogDetailWrapper<>::impl.startPollingThread(pollInterval);
}

template<int _>
void fmtlogT<_>::stopPollingThread() noexcept
{
  fmtlogDetailWrapper<>::impl.stopPollingThread();
}

template class fmtlogT<0>;
