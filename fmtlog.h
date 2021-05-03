#pragma once
//#define FMT_HEADER_ONLY
#include "fmt/format.h"
#include <type_traits>
#include <vector>

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
  static void setFlushDelay(uint64_t ns);

  // If current msg has level >= flushLogLevel, flush will be triggered
  static void flushOn(LogLevel flushLogLevel);

  // callback signature user can register
  // ns: nanosecond timestamp
  // level: logLevel
  // location: full file path with line num, e.g: /home/raomeng/fmtlog/fmtlog.h:45
  // basePos: file base index in the location
  // threadName: thread id or the name user set with setThreadName
  // msg: full log msg with header
  // bodyPos: log body index in the msg
  typedef void (*LogCBFn)(uint64_t ns, LogLevel level, fmt::string_view location, size_t basePos,
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
  static void startPollingThread(uint64_t pollInterval = 1000000);

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

  static void registerLogInfo(int& logId, FormatToFn fn, const char* location, LogLevel level,
                              fmt::string_view fmtString);

  // https://github.com/MengRao/SPSC_Queue
  template<uint32_t Bytes = 1 << 20>
  class SPSCVarQueueOPT
  {
  public:
    struct MsgHeader
    {
      uint32_t size;
      int logId;
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
      blk[write_idx].size = size;
      std::atomic_thread_fence(std::memory_order_release);
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
        return calibrate();
      }
    }

    double calibrate(uint64_t min_wait_ns = 10000000) {
      uint64_t delayed_tsc, delayed_ns;
      do {
        syncTime(delayed_tsc, delayed_ns);
      } while ((delayed_ns - base_ns) < min_wait_ns);
      tsc_ghz_inv = (double)(int64_t)(delayed_ns - base_ns) / (int64_t)(delayed_tsc - base_tsc);
      adjustOffset();
      return 1.0 / tsc_ghz_inv;
    }

    static inline uint64_t rdtsc() { return __builtin_ia32_rdtsc(); }

    inline uint64_t tsc2ns(uint64_t tsc) const { return ns_offset + (int64_t)((int64_t)tsc * tsc_ghz_inv); }

    inline uint64_t rdns() const { return tsc2ns(rdtsc()); }

    // If you want cross-platform, use std::chrono as below which incurs one more function call:
    // return std::chrono::high_resolution_clock::now().time_since_epoch().count();
    static uint64_t rdsysns() {
      timespec ts;
      ::clock_gettime(CLOCK_REALTIME, &ts);
      return ts.tv_sec * 1000000000 + ts.tv_nsec;
    }

    // For checking purposes, see test.cc
    uint64_t rdoffset() const { return ns_offset; }

  private:
    // Linux kernel sync time by finding the first try with tsc diff < 50000
    // We do better: we find the try with the mininum tsc diff
    void syncTime(uint64_t& tsc, uint64_t& ns) {
      const int N = 10;
      uint64_t tscs[N + 1];
      uint64_t nses[N + 1];

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

    void adjustOffset() { ns_offset = base_ns - (int64_t)((int64_t)base_tsc * tsc_ghz_inv); }

    alignas(64) double tsc_ghz_inv; // make sure tsc_ghz_inv and ns_offset are on the same cache line
    uint64_t ns_offset;
    uint64_t base_tsc;
    uint64_t base_ns;
  };

  bool inited = false;

public:
  TSCNS tscns;

private:
  volatile LogLevel currentLogLevel;
  static FAST_THREAD_LOCAL ThreadBuffer* threadBuffer;

  template<typename Arg>
  static inline constexpr bool isNamedArg() {
    return fmt::detail::is_named_arg<typename std::remove_reference<Arg>::type>::value;
  }

  template<typename Arg>
  struct unNamedType
  { using type = Arg; };

  template<typename Arg>
  struct unNamedType<fmt::detail::named_arg<char, Arg>>
  { using type = Arg; };

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
    using ArgType = typename std::remove_reference<Arg>::type;
    if constexpr (isCstring<Arg>()) return false;
    if constexpr (isString<Arg>()) return false;
    if constexpr (isNamedArg<Arg>()) {
      return !std::is_trivially_destructible<typename unNamedType<ArgType>::type>::value;
    }
    else {
      return !std::is_trivially_destructible<ArgType>::value;
    }
  }

  template<size_t CstringIdx>
  static inline constexpr size_t getArgSizes(size_t* cstringSize) {
    return 0;
  }

  template<size_t CstringIdx, typename Arg, typename... Args>
  static inline constexpr size_t getArgSizes(size_t* cstringSize, const Arg& arg, const Args&... args) {
    // fmt::print("type: {}, id: {}\n", type_name<Arg>(), (int)fmt::detail::mapped_type_constant<Arg, Context>::value);
    if constexpr (isNamedArg<Arg>()) {
      return getArgSizes<CstringIdx>(cstringSize, arg.value, args...);
    }
    else if constexpr (isCstring<Arg>()) {
      size_t len = strlen(arg) + 1;
      cstringSize[CstringIdx] = len;
      //  fmt::print("size: {}\n", len);
      return len + getArgSizes<CstringIdx + 1>(cstringSize, args...);
    }
    else if constexpr (isString<Arg>()) {
      size_t len = arg.size() + 1;
      // fmt::print("size: {}\n", len);
      return len + getArgSizes<CstringIdx>(cstringSize, args...);
    }
    else {
      // fmt::print("size: {}\n", sizeof(Arg));
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
      new (out) typename std::remove_reference<Arg>::type(std::forward<Arg>(arg));
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
    using ArgType = typename std::remove_reference<Arg>::type;
    if constexpr (isNamedArg<ArgType>()) {
      return decodeArgs<ValueOnly, Idx, DestructIdx, typename unNamedType<ArgType>::type, Args...>(in, args,
                                                                                                   destruct_args);
    }
    else if constexpr (isCstring<ArgType>() || isString<ArgType>()) {
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
        value_ = fmt::detail::arg_mapper<Context>().map(*(ArgType*)in);
      }
      else {
        args[Idx] = fmt::detail::make_arg<Context>(*(ArgType*)in);
      }

      if constexpr (needCallDtor<ArgType>()) {
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
    using ArgType = typename std::remove_reference<Arg>::type;
    if constexpr (isNamedArg<ArgType>()) {
      destructArgs<DestructIdx, typename unNamedType<ArgType>::type, Args...>(destruct_args);
    }
    else if constexpr (needCallDtor<ArgType>()) {
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
    char* dtor_args[num_dtors];
    char* ret;
    if (argIdx < 0) {
      argIdx = args.size();
      args.resize(argIdx + num_args);
      ret = decodeArgs<false, 0, 0, Args...>(data, &args[argIdx], dtor_args);
    }
    else {
      ret = decodeArgs<true, 0, 0, Args...>(data, &args[argIdx], dtor_args);
    }
    fmt::detail::vformat_to(out, format, fmt::basic_format_args(&args[argIdx], num_args));
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
    fmt::detail::named_arg_info<char> named_args[num_named_args];
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
  inline void log(int& logId, uint64_t tsc, const char* location, LogLevel level, const S& format, Args&&... args) {
    constexpr size_t num_named_args = fmt::detail::count<isNamedArg<Args>()...>();
    if constexpr (num_named_args == 0) {
      fmt::detail::check_format_string<Args...>(format);
    }
    if (logId < 0) {
      auto unnamed_format = unNameFormat<false>(fmt::to_string_view(format), nullptr, args...);
      registerLogInfo(logId, formatTo<Args...>, location, level, unnamed_format);
    }
    constexpr size_t num_cstring = fmt::detail::count<isCstring<Args>()...>();
    size_t cstringSizes[num_cstring];
    size_t allocSize = getArgSizes<0>(cstringSizes, args...) + 8;
    if (threadBuffer == nullptr) preallocate();
    do {
      if (threadBuffer->varq.tryPush(allocSize, [&](typename SPSCVarQueueOPT<>::MsgHeader* header) {
            header->logId = logId;
            char* writePos = (char*)(header + 1);
            *(uint64_t*)writePos = tsc;
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
    size_t formatted_size = fmt::formatted_size(sv, args...);
    size_t allocSize = formatted_size + 8 + 4 + 8;
    if (threadBuffer == nullptr) preallocate();
    do {
      if (threadBuffer->varq.tryPush(allocSize, [&](typename SPSCVarQueueOPT<>::MsgHeader* header) {
            header->logId = -1 - (int)level;
            char* writePos = (char*)(header + 1);
            *(const char**)writePos = location;
            writePos += 8;
            *(uint64_t*)writePos = tscns.rdtsc();
            writePos += 8;
            fmt::format_to(writePos, sv, args...);
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
    static int logId = -1;                                                                                             \
                                                                                                                       \
    if (level < fmtlog::getLogLevel()) break;                                                                          \
                                                                                                                       \
    fmtlogWrapper<>::impl.log(logId, fmtlogWrapper<>::impl.tscns.rdtsc(), __FMTLOG_LOCATION, level,                    \
                              FMT_STRING(format), ##__VA_ARGS__);                                                      \
  } while (0)

#define FMTLOG_LIMIT(min_interval, level, format, ...)                                                                 \
  do {                                                                                                                 \
    static int logId = -1;                                                                                             \
    static uint64_t limitNs = 0;                                                                                       \
                                                                                                                       \
    if (level < fmtlog::getLogLevel()) break;                                                                          \
    uint64_t tsc = fmtlogWrapper<>::impl.tscns.rdtsc();                                                                \
    uint64_t ns = fmtlogWrapper<>::impl.tscns.tsc2ns(tsc);                                                             \
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
