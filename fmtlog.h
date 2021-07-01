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
//#define FMT_HEADER_ONLY
#include "fmt/format.h"
#include <type_traits>
#include <vector>
#include <chrono>
#include <memory>

#ifdef _WIN32
#define FAST_THREAD_LOCAL thread_local
#else
#define FAST_THREAD_LOCAL __thread
#endif

// define FMTLOG_BLOCK=1 if log statment should be blocked when queue is full, instead of discarding the msg
#ifndef FMTLOG_BLOCK
#define FMTLOG_BLOCK 0
#endif

#define FMTLOG_LEVEL_DBG 0
#define FMTLOG_LEVEL_INF 1
#define FMTLOG_LEVEL_WRN 2
#define FMTLOG_LEVEL_ERR 3
#define FMTLOG_LEVEL_OFF 4

// define FMTLOG_ACTIVE_LEVEL to turn off low log level in compile time
#ifndef FMTLOG_ACTIVE_LEVEL
#define FMTLOG_ACTIVE_LEVEL FMTLOG_LEVEL_INF
#endif

namespace fmtlogdetail {
template<typename Arg>
struct UnrefPtr : std::false_type
{ using type = Arg; };

template<>
struct UnrefPtr<char*> : std::false_type
{ using type = char*; };

template<>
struct UnrefPtr<void*> : std::false_type
{ using type = void*; };

template<typename Arg>
struct UnrefPtr<std::shared_ptr<Arg>> : std::true_type
{ using type = Arg; };

template<typename Arg, typename D>
struct UnrefPtr<std::unique_ptr<Arg, D>> : std::true_type
{ using type = Arg; };

template<typename Arg>
struct UnrefPtr<Arg*> : std::true_type
{ using type = Arg; };
}; // namespace fmtlogdetail

template<int __ = 0>
class fmtlogT
{
public:
  enum LogLevel : uint8_t
  {
    DBG = 0,
    INF,
    WRN,
    ERR,
    OFF
  };

  // If you know the exact tsc frequency(in ghz) in the os, tell fmtlog!
  // But how can I know the frequency? Check below link(for Linux only):
  // https://github.com/MengRao/tscns#i-dont-wanna-wait-a-long-time-for-calibration-can-i-cheat
  static void setTscGhz(double tscGhz);

  // Preallocate thread queue for current thread
  static void preallocate();

  // Set the file for logging
  static void setLogFile(const char* filename, bool truncate = false);

  // Set an existing FILE* for logging, if manageFp is false fmtlog will not buffer log internally and will not close
  // the FILE*
  static void setLogFile(FILE* fp, bool manageFp = false);

  // Collect log msgs from all threads and write to log file
  // If forceFlush = true, internal file buffer is flushed
  // User need to call poll() repeatedly if startPollingThread is not used
  static void poll(bool forceFlush = false);

  // Set flush delay in nanosecond
  // If there's msg older than ns in the buffer, flush will be triggered
  static void setFlushDelay(int64_t ns);

  // If current msg has level >= flushLogLevel, flush will be triggered
  static void flushOn(LogLevel flushLogLevel);

  // If file buffer has more than specified bytes, flush will be triggered
  static void setFlushBufSize(uint32_t bytes);

  // callback signature user can register
  // ns: nanosecond timestamp
  // level: logLevel
  // location: full file path with line num, e.g: /home/raomeng/fmtlog/fmtlog.h:45
  // basePos: file base index in the location
  // threadName: thread id or the name user set with setThreadName
  // msg: full log msg with header
  // bodyPos: log body index in the msg
  typedef void (*LogCBFn)(int64_t ns, LogLevel level, fmt::string_view location, size_t basePos,
                          fmt::string_view threadName, fmt::string_view msg, size_t bodyPos);

  // Set a callback function for all log msgs with a mininum log level
  static void setLogCB(LogCBFn cb, LogLevel minCBLogLevel);

  // Close the log file and subsequent msgs will not be written into the file,
  // but callback function can still be used
  static void closeLogFile();

  // Set log header pattern with fmt named arguments
  static void setHeaderPattern(const char* pattern);

  // Set a name for current thread, it'll be shown in {t} part in header pattern
  static void setThreadName(const char* name);

  // Set current log level, lower level log msgs will be discarded
  static inline void setLogLevel(LogLevel logLevel);

  // Get current log level
  static inline LogLevel getLogLevel();

  // Run a polling thread in the background with a polling interval
  // Note that user must not call poll() himself when the thread is running
  static void startPollingThread(int64_t pollInterval = 1000000);

  // Stop the polling thread
  static void stopPollingThread();

private:
  fmtlogT() { init(); }

  void init() {
    if (!inited) {
      inited = true;
      tscns.init();
      currentLogLevel = INF;
    }
  }

  template<int>
  friend class fmtlogDetailT;
  template<int>
  friend struct fmtlogWrapper;
  template<typename S, typename... Args>
  friend void test(const S& format, Args&&...);

  using Context = fmt::format_context;
  using MemoryBuffer = fmt::basic_memory_buffer<char, 10000>;
  typedef char* (*FormatToFn)(fmt::string_view format, char* data, MemoryBuffer& out, int& argIdx,
                              std::vector<fmt::basic_format_arg<Context>>& args);

  static void registerLogInfo(uint32_t& logId, FormatToFn fn, const char* location, LogLevel level,
                              fmt::string_view fmtString);

  // https://github.com/MengRao/SPSC_Queue
  template<uint32_t Bytes = 1 << 20>
  class SPSCVarQueueOPT
  {
  public:
    struct MsgHeader
    {
      uint32_t size;
      uint32_t logId;
    };
    static constexpr uint32_t BLK_CNT = Bytes / sizeof(MsgHeader);

    MsgHeader* alloc(uint32_t size_) {
      size = size_ + sizeof(MsgHeader);
      uint32_t blk_sz = (size + sizeof(MsgHeader) - 1) / sizeof(MsgHeader);
      if (blk_sz >= free_write_cnt) {
        uint32_t read_idx_cache = *(volatile uint32_t*)&read_idx;
        if (read_idx_cache <= write_idx) {
          free_write_cnt = BLK_CNT - write_idx;
          if (blk_sz >= free_write_cnt && read_idx_cache != 0) { // wrap around
            blk[0].size = 0;
            std::atomic_thread_fence(std::memory_order_release);
            blk[write_idx].size = 1;
            write_idx = 0;
            free_write_cnt = read_idx_cache;
          }
        }
        else {
          free_write_cnt = read_idx_cache - write_idx;
        }
        if (free_write_cnt <= blk_sz) {
          return nullptr;
        }
      }
      return &blk[write_idx];
    }

    void push() {
      uint32_t blk_sz = (size + sizeof(MsgHeader) - 1) / sizeof(MsgHeader);
      blk[write_idx + blk_sz].size = 0;
      std::atomic_thread_fence(std::memory_order_release);
      blk[write_idx].size = size;
      write_idx += blk_sz;
      free_write_cnt -= blk_sz;
    }

    template<typename Writer>
    bool tryPush(uint32_t size, Writer writer) {
      MsgHeader* header = alloc(size);
      if (!header) return false;
      writer(header);
      push();
      return true;
    }

    MsgHeader* front() {
      uint32_t size = blk[read_idx].size;
      if (size == 1) { // wrap around
        read_idx = 0;
        size = blk[0].size;
      }
      if (size == 0) return nullptr;
      return &blk[read_idx];
    }

    void pop() {
      uint32_t blk_sz = (blk[read_idx].size + sizeof(MsgHeader) - 1) / sizeof(MsgHeader);
      *(volatile uint32_t*)&read_idx = read_idx + blk_sz;
    }

    template<typename Reader>
    bool tryPop(Reader reader) {
      MsgHeader* header = front();
      if (!header) return false;
      reader(header);
      pop();
      return true;
    }

  private:
    alignas(64) MsgHeader blk[BLK_CNT] = {};

    alignas(128) uint32_t write_idx = 0;
    uint32_t free_write_cnt = BLK_CNT;
    uint32_t size;

    alignas(128) uint32_t read_idx = 0;
  };

  struct ThreadBuffer
  {
    SPSCVarQueueOPT<> varq;
    bool shouldDeallocate = false;
    char name[32];
    size_t nameSize;
  };

  // https://github.com/MengRao/tscns
  class TSCNS
  {
  public:
    double init(double tsc_ghz = 0.0) {
      syncTime(base_tsc, base_ns);
      if (tsc_ghz > 0) {
        tsc_ghz_inv = 1.0 / tsc_ghz;
        adjustOffset();
        return tsc_ghz;
      }
      else {
#ifdef _WIN32
        return calibrate(1000000 * 100); // wait more time as Windows' system time is in 100ns precision
#else
        return calibrate(1000000 * 10); //
#endif
      }
    }

    double calibrate(int64_t min_wait_ns) {
      int64_t delayed_tsc, delayed_ns;
      do {
        syncTime(delayed_tsc, delayed_ns);
      } while ((delayed_ns - base_ns) < min_wait_ns);
      tsc_ghz_inv = (double)(delayed_ns - base_ns) / (delayed_tsc - base_tsc);
      adjustOffset();
      return 1.0 / tsc_ghz_inv;
    }

    static inline int64_t rdtsc() {
#ifdef _WIN32
      return __rdtsc();
#else
      return __builtin_ia32_rdtsc();
#endif
    }

    inline int64_t tsc2ns(int64_t tsc) const { return ns_offset + (int64_t)(tsc * tsc_ghz_inv); }

    inline int64_t rdns() const { return tsc2ns(rdtsc()); }

    static int64_t rdsysns() {
      using namespace std::chrono;
      return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
    }

    // For checking purposes, see test.cc
    int64_t rdoffset() const { return ns_offset; }

  private:
    // Linux kernel sync time by finding the first try with tsc diff < 50000
    // We do better: we find the try with the mininum tsc diff
    void syncTime(int64_t& tsc, int64_t& ns) {
      const int N = 10;
      int64_t tscs[N + 1];
      int64_t nses[N + 1];

      tscs[0] = rdtsc();
      for (int i = 1; i <= N; i++) {
        nses[i] = rdsysns();
        tscs[i] = rdtsc();
      }

      int best = 1;
      for (int i = 2; i <= N; i++) {
        if (tscs[i] - tscs[i - 1] < tscs[best] - tscs[best - 1]) best = i;
      }
      tsc = (tscs[best] + tscs[best - 1]) >> 1;
      ns = nses[best];
    }

    void adjustOffset() { ns_offset = base_ns - (int64_t)(base_tsc * tsc_ghz_inv); }

    alignas(64) double tsc_ghz_inv; // make sure tsc_ghz_inv and ns_offset are on the same cache line
    int64_t ns_offset;
    int64_t base_tsc;
    int64_t base_ns;
  };

  bool inited = false;

public:
  TSCNS tscns;

private:
  volatile LogLevel currentLogLevel;
  static FAST_THREAD_LOCAL ThreadBuffer* threadBuffer;

  template<typename Arg>
  static inline constexpr bool isNamedArg() {
    return fmt::detail::is_named_arg<fmt::remove_cvref_t<Arg>>::value;
  }

  template<typename Arg>
  struct unNamedType
  { using type = Arg; };

  template<typename Arg>
  struct unNamedType<fmt::detail::named_arg<char, Arg>>
  { using type = Arg; };

#if FMT_USE_NONTYPE_TEMPLATE_PARAMETERS
  template<typename Arg, size_t N, fmt::detail_exported::fixed_string<char, N> Str>
  struct unNamedType<fmt::detail::statically_named_arg<Arg, char, N, Str>>
  { using type = Arg; };
#endif


  template<typename Arg>
  static inline constexpr bool isCstring() {
    return fmt::detail::mapped_type_constant<Arg, Context>::value == fmt::detail::type::cstring_type;
  }

  template<typename Arg>
  static inline constexpr bool isString() {
    return fmt::detail::mapped_type_constant<Arg, Context>::value == fmt::detail::type::string_type;
  }

  template<typename Arg>
  static inline constexpr bool needCallDtor() {
    using ArgType = fmt::remove_cvref_t<Arg>;
    if constexpr (isNamedArg<Arg>()) {
      return needCallDtor<typename unNamedType<ArgType>::type>();
    }
    if constexpr (isString<Arg>()) return false;
    return !std::is_trivially_destructible<ArgType>::value;
  }

  template<size_t CstringIdx>
  static inline constexpr size_t getArgSizes(size_t* cstringSize) {
    return 0;
  }

  template<size_t CstringIdx, typename Arg, typename... Args>
  static inline constexpr size_t getArgSizes(size_t* cstringSize, const Arg& arg, const Args&... args) {
    if constexpr (isNamedArg<Arg>()) {
      return getArgSizes<CstringIdx>(cstringSize, arg.value, args...);
    }
    else if constexpr (isCstring<Arg>()) {
      size_t len = strlen(arg) + 1;
      cstringSize[CstringIdx] = len;
      return len + getArgSizes<CstringIdx + 1>(cstringSize, args...);
    }
    else if constexpr (isString<Arg>()) {
      size_t len = arg.size() + 1;
      return len + getArgSizes<CstringIdx>(cstringSize, args...);
    }
    else {
      return sizeof(Arg) + getArgSizes<CstringIdx>(cstringSize, args...);
    }
  }

  template<size_t CstringIdx>
  static inline constexpr char* encodeArgs(size_t* cstringSize, char* out) {
    return out;
  }

  template<size_t CstringIdx, typename Arg, typename... Args>
  static inline constexpr char* encodeArgs(size_t* cstringSize, char* out, Arg&& arg, Args&&... args) {
    if constexpr (isNamedArg<Arg>()) {
      return encodeArgs<CstringIdx>(cstringSize, out, arg.value, std::forward<Args>(args)...);
    }
    else if constexpr (isCstring<Arg>()) {
      memcpy(out, arg, cstringSize[CstringIdx]);
      return encodeArgs<CstringIdx + 1>(cstringSize, out + cstringSize[CstringIdx], std::forward<Args>(args)...);
    }
    else if constexpr (isString<Arg>()) {
      size_t len = arg.size();
      memcpy(out, arg.data(), len);
      out[len] = 0;
      return encodeArgs<CstringIdx>(cstringSize, out + len + 1, std::forward<Args>(args)...);
    }
    else {
      new (out) fmt::remove_cvref_t<Arg>(std::forward<Arg>(arg));
      return encodeArgs<CstringIdx>(cstringSize, out + sizeof(Arg), std::forward<Args>(args)...);
    }
  }

  template<size_t Idx, size_t NamedIdx>
  static inline constexpr void storeNamedArgs(fmt::detail::named_arg_info<char>* named_args_store) {}

  template<size_t Idx, size_t NamedIdx, typename Arg, typename... Args>
  static inline constexpr void storeNamedArgs(fmt::detail::named_arg_info<char>* named_args_store, const Arg& arg,
                                              const Args&... args) {
    if constexpr (isNamedArg<Arg>()) {
      named_args_store[NamedIdx] = {arg.name, Idx};
      storeNamedArgs<Idx + 1, NamedIdx + 1>(named_args_store, args...);
    }
    else {
      storeNamedArgs<Idx + 1, NamedIdx>(named_args_store, args...);
    }
  }

  template<bool ValueOnly, size_t Idx, size_t DestructIdx>
  static inline char* decodeArgs(char* in, fmt::basic_format_arg<Context>* args, char** destruct_args) {
    return in;
  }

  template<bool ValueOnly, size_t Idx, size_t DestructIdx, typename Arg, typename... Args>
  static inline char* decodeArgs(char* in, fmt::basic_format_arg<Context>* args, char** destruct_args) {
    using namespace fmtlogdetail;
    using ArgType = fmt::remove_cvref_t<Arg>;
    if constexpr (isNamedArg<ArgType>()) {
      return decodeArgs<ValueOnly, Idx, DestructIdx, typename unNamedType<ArgType>::type, Args...>(in, args,
                                                                                                   destruct_args);
    }
    else if constexpr (isCstring<Arg>() || isString<Arg>()) {
      size_t size = strlen(in);
      fmt::string_view v(in, size);
      if constexpr (ValueOnly) {
        fmt::detail::value<Context>& value_ = *(fmt::detail::value<Context>*)(args + Idx);
        value_ = fmt::detail::arg_mapper<Context>().map(v);
      }
      else {
        args[Idx] = fmt::detail::make_arg<Context>(v);
      }
      return decodeArgs<ValueOnly, Idx + 1, DestructIdx, Args...>(in + size + 1, args, destruct_args);
    }
    else {
      if constexpr (ValueOnly) {
        fmt::detail::value<Context>& value_ = *(fmt::detail::value<Context>*)(args + Idx);
        if constexpr (UnrefPtr<ArgType>::value) {
          value_ = fmt::detail::arg_mapper<Context>().map(**(ArgType*)in);
        }
        else {
          value_ = fmt::detail::arg_mapper<Context>().map(*(ArgType*)in);
        }
      }
      else {
        if constexpr (UnrefPtr<ArgType>::value) {
          args[Idx] = fmt::detail::make_arg<Context>(**(ArgType*)in);
        }
        else {
          args[Idx] = fmt::detail::make_arg<Context>(*(ArgType*)in);
        }
      }

      if constexpr (needCallDtor<Arg>()) {
        destruct_args[DestructIdx] = in;
        return decodeArgs<ValueOnly, Idx + 1, DestructIdx + 1, Args...>(in + sizeof(ArgType), args, destruct_args);
      }
      else {
        return decodeArgs<ValueOnly, Idx + 1, DestructIdx, Args...>(in + sizeof(ArgType), args, destruct_args);
      }
    }
  }

  template<size_t DestructIdx>
  static inline void destructArgs(char** destruct_args) {}

  template<size_t DestructIdx, typename Arg, typename... Args>
  static inline void destructArgs(char** destruct_args) {
    using ArgType = fmt::remove_cvref_t<Arg>;
    if constexpr (isNamedArg<ArgType>()) {
      destructArgs<DestructIdx, typename unNamedType<ArgType>::type, Args...>(destruct_args);
    }
    else if constexpr (needCallDtor<Arg>()) {
      ((ArgType*)destruct_args[DestructIdx])->~ArgType();
      destructArgs<DestructIdx + 1, Args...>(destruct_args);
    }
    else {
      destructArgs<DestructIdx, Args...>(destruct_args);
    }
  }

  template<typename... Args>
  static char* formatTo(fmt::string_view format, char* data, MemoryBuffer& out, int& argIdx,
                        std::vector<fmt::basic_format_arg<Context>>& args) {
    constexpr size_t num_args = sizeof...(Args);
    constexpr size_t num_dtors = fmt::detail::count<needCallDtor<Args>()...>();
    char* dtor_args[std::max(num_dtors, (size_t)1)];
    char* ret;
    if (argIdx < 0) {
      argIdx = args.size();
      args.resize(argIdx + num_args);
      ret = decodeArgs<false, 0, 0, Args...>(data, args.data() + argIdx, dtor_args);
    }
    else {
      ret = decodeArgs<true, 0, 0, Args...>(data, args.data() + argIdx, dtor_args);
    }
    fmt::detail::vformat_to(out, format, fmt::basic_format_args(args.data() + argIdx, num_args));
    destructArgs<0, Args...>(dtor_args);

    return ret;
  }

  template<bool Reorder, typename... Args>
  static constexpr fmt::string_view unNameFormat(fmt::string_view in, uint32_t* reorderIdx, const Args&... args) {
    constexpr size_t num_named_args = fmt::detail::count<isNamedArg<Args>()...>();
    if constexpr (num_named_args == 0) {
      return in;
    }
    const char* begin = in.data();
    const char* p = begin;
    std::unique_ptr<char[]> unnamed_str(new char[in.size() + 1 + num_named_args * 5]);
    fmt::detail::named_arg_info<char> named_args[std::max(num_named_args, (size_t)1)];
    storeNamedArgs<0, 0>(named_args, args...);

    char* out = (char*)unnamed_str.get();
    uint8_t arg_idx = 0;
    while (true) {
      auto c = *p++;
      if (!c) {
        size_t copy_size = p - begin - 1;
        memcpy(out, begin, copy_size);
        out += copy_size;
        break;
      }
      if (c != '{') continue;
      size_t copy_size = p - begin;
      memcpy(out, begin, copy_size);
      out += copy_size;
      begin = p;
      c = *p++;
      if (!c) throw std::runtime_error("invalid format string");
      if (fmt::detail::is_name_start(c)) {
        while ((fmt::detail::is_name_start(c = *p) || ('0' <= c && c <= '9'))) {
          ++p;
        }
        fmt::string_view name(begin, p - begin);
        int id = -1;
        for (size_t i = 0; i < num_named_args; ++i) {
          if (named_args[i].name == name) {
            id = named_args[i].id;
            break;
          }
        }
        if (id < 0) throw std::runtime_error("invalid format string");
        if constexpr (Reorder) {
          reorderIdx[id] = arg_idx++;
        }
        else {
          out = fmt::format_to(out, "{}", id);
        }
      }
      else {
        *out++ = c;
      }
      begin = p;
    }
    const char* ptr = unnamed_str.release();
    return fmt::string_view(ptr, out - ptr);
  }

public:
  template<typename S, typename... Args>
  inline void log(uint32_t& logId, int64_t tsc, const char* location, LogLevel level, const S& format, Args&&... args) {
    using namespace fmtlogdetail;
    constexpr size_t num_named_args = fmt::detail::count<isNamedArg<Args>()...>();
    if constexpr (num_named_args == 0) {
      fmt::detail::check_format_string<typename UnrefPtr<fmt::remove_cvref_t<Args>>::type...>(format);
    }
    if (!logId) {
      auto unnamed_format = unNameFormat<false>(fmt::to_string_view(format), nullptr, args...);
      registerLogInfo(logId, formatTo<Args...>, location, level, unnamed_format);
    }
    constexpr size_t num_cstring = fmt::detail::count<isCstring<Args>()...>();
    size_t cstringSizes[std::max(num_cstring, (size_t)1)];
    size_t allocSize = getArgSizes<0>(cstringSizes, args...) + 8;
    if (threadBuffer == nullptr) preallocate();
    do {
      if (threadBuffer->varq.tryPush(allocSize, [&](typename SPSCVarQueueOPT<>::MsgHeader* header) {
            header->logId = logId;
            char* writePos = (char*)(header + 1);
            *(int64_t*)writePos = tsc;
            writePos += 8;
            encodeArgs<0>(cstringSizes, writePos, std::forward<Args>(args)...);
          }))
        return;
    } while (FMTLOG_BLOCK);
  }

  template<typename S, typename... Args>
  inline void logOnce(const char* location, LogLevel level, const S& format, Args&&... args) {
    constexpr size_t num_named_args = fmt::detail::count<isNamedArg<Args>()...>();
    if constexpr (num_named_args == 0) {
      fmt::detail::check_format_string<Args...>(format);
    }
    fmt::string_view sv(format);
    size_t formatted_size = fmt::formatted_size(fmt::runtime(sv), args...);
    size_t allocSize = formatted_size + 8 + 8;
    if (threadBuffer == nullptr) preallocate();
    do {
      if (threadBuffer->varq.tryPush(allocSize, [&](typename SPSCVarQueueOPT<>::MsgHeader* header) {
            header->logId = (uint32_t)level;
            char* writePos = (char*)(header + 1);
            *(const char**)writePos = location;
            writePos += 8;
            *(int64_t*)writePos = tscns.rdtsc();
            writePos += 8;
            fmt::format_to(writePos, fmt::runtime(sv), args...);
          }))
        return;
    } while (FMTLOG_BLOCK);
  }
};

using fmtlog = fmtlogT<>;

template<int _>
FAST_THREAD_LOCAL typename fmtlogT<_>::ThreadBuffer* fmtlogT<_>::threadBuffer;

template<int __ = 0>
struct fmtlogWrapper
{ static fmtlog impl; };

template<int _>
fmtlog fmtlogWrapper<_>::impl;

template<int _>
inline void fmtlogT<_>::setLogLevel(LogLevel logLevel) {
  fmtlogWrapper<>::impl.currentLogLevel = logLevel;
}

template<int _>
inline typename fmtlogT<_>::LogLevel fmtlogT<_>::getLogLevel() {
  return fmtlogWrapper<>::impl.currentLogLevel;
}

#define __FMTLOG_S1(x) #x
#define __FMTLOG_S2(x) __FMTLOG_S1(x)
#define __FMTLOG_LOCATION __FILE__ ":" __FMTLOG_S2(__LINE__)

#define FMTLOG(level, format, ...)                                                                                     \
  do {                                                                                                                 \
    static uint32_t logId = 0;                                                                                         \
                                                                                                                       \
    if (level < fmtlog::getLogLevel()) break;                                                                          \
                                                                                                                       \
    fmtlogWrapper<>::impl.log(logId, fmtlogWrapper<>::impl.tscns.rdtsc(), __FMTLOG_LOCATION, level,                    \
                              FMT_STRING(format), ##__VA_ARGS__);                                                      \
  } while (0)

#define FMTLOG_LIMIT(min_interval, level, format, ...)                                                                 \
  do {                                                                                                                 \
    static uint32_t logId = 0;                                                                                         \
    static int64_t limitNs = 0;                                                                                        \
                                                                                                                       \
    if (level < fmtlog::getLogLevel()) break;                                                                          \
    int64_t tsc = fmtlogWrapper<>::impl.tscns.rdtsc();                                                                 \
    int64_t ns = fmtlogWrapper<>::impl.tscns.tsc2ns(tsc);                                                              \
    if (ns < limitNs) break;                                                                                           \
    limitNs = ns + min_interval;                                                                                       \
                                                                                                                       \
    fmtlogWrapper<>::impl.log(logId, tsc, __FMTLOG_LOCATION, level, FMT_STRING(format), ##__VA_ARGS__);                \
  } while (0)

#define FMTLOG_ONCE(level, format, ...)                                                                                \
  do {                                                                                                                 \
    if (level < fmtlog::getLogLevel()) break;                                                                          \
                                                                                                                       \
    fmtlogWrapper<>::impl.logOnce(__FMTLOG_LOCATION, level, FMT_STRING(format), ##__VA_ARGS__);                        \
  } while (0)

#if FMTLOG_ACTIVE_LEVEL <= FMTLOG_LEVEL_DBG
#define logd(format, ...) FMTLOG(fmtlog::DBG, format, ##__VA_ARGS__)
#define logdo(format, ...) FMTLOG_ONCE(fmtlog::DBG, format, ##__VA_ARGS__)
#define logdl(min_interval, format, ...) FMTLOG_LIMIT(min_interval, fmtlog::DBG, format, ##__VA_ARGS__)
#else
#define logd(format, ...) (void)0
#define logdo(format, ...) (void)0
#define logdl(min_interval, format, ...) (void)0
#endif

#if FMTLOG_ACTIVE_LEVEL <= FMTLOG_LEVEL_INF
#define logi(format, ...) FMTLOG(fmtlog::INF, format, ##__VA_ARGS__)
#define logio(format, ...) FMTLOG_ONCE(fmtlog::INF, format, ##__VA_ARGS__)
#define logil(min_interval, format, ...) FMTLOG_LIMIT(min_interval, fmtlog::INF, format, ##__VA_ARGS__)
#else
#define logi(format, ...) (void)0
#define logio(format, ...) (void)0
#define logil(min_interval, format, ...) (void)0
#endif

#if FMTLOG_ACTIVE_LEVEL <= FMTLOG_LEVEL_WRN
#define logw(format, ...) FMTLOG(fmtlog::WRN, format, ##__VA_ARGS__)
#define logwo(format, ...) FMTLOG_ONCE(fmtlog::WRN, format, ##__VA_ARGS__)
#define logwl(min_interval, format, ...) FMTLOG_LIMIT(min_interval, fmtlog::WRN, format, ##__VA_ARGS__)
#else
#define logw(format, ...) (void)0
#define logwo(format, ...) (void)0
#define logwl(min_interval, format, ...) (void)0
#endif

#if FMTLOG_ACTIVE_LEVEL <= FMTLOG_LEVEL_ERR
#define loge(format, ...) FMTLOG(fmtlog::ERR, format, ##__VA_ARGS__)
#define logeo(format, ...) FMTLOG_ONCE(fmtlog::ERR, format, ##__VA_ARGS__)
#define logel(min_interval, format, ...) FMTLOG_LIMIT(min_interval, fmtlog::ERR, format, ##__VA_ARGS__)
#else
#define loge(format, ...) (void)0
#define logeo(format, ...) (void)0
#define logel(min_interval, format, ...) (void)0
#endif

#ifdef FMTLOG_HEADER_ONLY
#include "fmtlog-inl.h"
#endif
