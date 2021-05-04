# fmtlog
fmtlog is an asynchronous logging library using [fmt](https://github.com/fmtlib/fmt) library format. It's extremely performant with runtime latency lower than [NanoLog](https://github.com/PlatformLab/NanoLog) and throughput higher than [spdlog](https://github.com/gabime/spdlog).

## Features
* Faster (see [Performance](https://github.com/MengRao/fmtlog#Performance) below)
* Headers only or compiled
* Feature rich formatting on top of excellent fmt library.
* Asynchronous multi-threaded logging and can also be used synchronously in single thread.
* Custom formatting
* Custom handling - user can set a callback function to handle log msgs in addition to writing into file.
* Log filtering - log levels can be modified in runtime as well as in compile time.
* Log frequency limitation - specific logs can be set a minimum logging interval.

## Install
C++17 is requried, and fmtlog is dependent on [fmtlib](https://github.com/fmtlib/fmt), you need to install fmtlib first if you haven't.
#### Header only version
Just copy `fmtlog.h` and `fmtlog-inl.h` to your project, and define macro `FMTLOG_HEADER_ONLY` before including fmtlog.h. You might also want to define `FMT_HEADER_ONLY` if you are using fmtlib in header-only way.

#### Static/Shared lib version built by CMake
```console
$ git clone https://github.com/MengRao/fmtlog.git
$ cd fmtlog && ./build.sh
```
Then copy `fmtlog.h` and `libfmtlog-static.a` or `libfmtlog-shared.so` generated in `.build` dir.

## Usage
```c++
#include "fmtlog/fmtlog.h"
int main() 
{
  FMTLOG(fmtlog::INF, "The answer is {}.", 42);
}
```
There're also shortcut macros `logd`, `logi`, `logw` and `loge` defined for logging `DBG`, `INF`, `WRN` and `ERR` msgs respectively:
```c++
logi("A info msg");
logd("This msg will not be logged as the default log level is INF");
fmtlog::setLogLevel(fmtlog::DBG);
logd("Now debug msg is shown");
```
Note that fmtlog is asynchronous in nature, msgs are not written into file/console immediately after the log statements: they are simply pushed onto a queue. You need to call `fmtlog::poll()` to collect data from log queues, format and write it out:
```c++
fmtlog::setThreadName("aaa");
logi("Thread name is bbb in this msg");
fmtlog::setThreadName("bbb");
fmtlog::poll();
fmtlog::setThreadName("ccc");
logi("Thread name is ccc in this msg");
fmtlog::poll();
fmtlog::setThreadName("ddd");

```
fmtlog supports multi-threaded logging, but can only have one thread calling `fmtlog::poll()`. By default, fmtlog doesn't create a polling thread internally, it requires the user to poll it periodically. The idea is that this allows users to manage the threads in their own way, and have full control of polling/flushing behavior. However, you can ask  fmtlog to create a background polling thread for you by `fmtlog::startPollingThread(interval)` with a polling interval, but you can't call `fmtlog::poll()` when the thread is running.

## Format
fmtlog is based on fmtlib, almost all fmtlib features are supported(except for color):
```c++
#include "fmt/ranges.h"
using namespace fmt::literals;

logi("I'd rather be {1} than {0}.", "right", "happy");
logi("Hello, {name}! The answer is {number}. Goodbye, {name}.", "name"_a = "World", "number"_a = 42);

std::vector<int> v = {1, 2, 3};
logi("ranges: {}", v);

logi("std::move can be used for objects with non-trivial destructors: {}", std::move(v));
assert(v.size() == 0);

std::tuple<int, char> t = {1, 'a'};
logi("tuples: {}", fmt::join(t, ", "));

enum class color {red, green, blue};
template <> struct fmt::formatter<color>: formatter<string_view> {
  // parse is inherited from formatter<string_view>.
  template <typename FormatContext>
  auto format(color c, FormatContext& ctx) {
    string_view name = "unknown";
    switch (c) {
    case color::red:   name = "red"; break;
    case color::green: name = "green"; break;
    case color::blue:  name = "blue"; break;
    }
    return formatter<string_view>::format(name, ctx);
  }
};
logi("user defined type: {:>10}", color::blue);
logi("{:*^30}", "centered");
logi("int: {0:d};  hex: {0:#x};  oct: {0:#o};  bin: {0:#b}", 42);
logi("dynamic precision: {:.{}f}", 3.14, 1);

// A compile-time error because 'd' is an invalid specifier for strings.
logi("{:d}", "foo");
```
Log header pattern can also be customized with `fmtlog::setHeaderPattern()` and the argument is a fmtlib format string with named arguments. The default header pattern is "{HMSf} {s:<16} {l}[{t:<6}] " (example: "15:46:19.149844 log_test.cc:43   INF[448050] "). All supported named arguments in header are as below:
| name | meaning| example |
| :------ | :-------: | :-----: |
|`l`|Log level|INF|
|`s`|File base name and line num|log_test.cc:48|
|`g`|File path and line num|/home/raomeng/fmt/log_test.cc:48|
|`t`|Thread id by default, can be reset by `fmt::setThreadName()`|main|
|`a`|Weekday|Mon|
|`b`|Month name|May|
|`Y`|Year|2021|
|`C`|Short year|21|
|`m`|Month|05|
|`d`|Day|03|
|`H`|Hour|16|
|`M`|Minute|08|
|`S`|Second|09|
|`e`|Millisecond|796|
|`f`|Microsecond|796341|
|`F`|Nanosecond|796341126|
|`Ymd`|Year-Month-Day|2021-05-03|
|`HMS`|Hour:Minute:Second|16:08:09|
|`HMSe`|Hour:Minute:Second.Millisecond|16:08:09.796|
|`HMSf`|Hour:Minute:Second.Microsecond|16:08:09.796341|
|`HMSF`|Hour:Minute:Second.Nanosecond|16:08:09.796341126|
|`YmdHMS`|Year-Month-Day Hour:Minute:Second|2021-05-03 16:08:09|
|`YmdHMSe`|Year-Month-Day Hour:Minute:Second.Millisecond|2021-05-03 16:08:09.796|
|`YmdHMSf`|Year-Month-Day Hour:Minute:Second.Microsecond|2021-05-03 16:08:09.796341|
|`YmdHMSF`|Year-Month-Day Hour:Minute:Second.Nanosecond|2021-05-03 16:08:09.796341126|

Note that using concatenated named args is more efficient than seperated ones, e.g. `{YmdHMS}` is faster than `{Y}-{m}-{d} {H}:{M}:{S}`.

## Output
By default, fmtlog output to stdout. Normally users want to write to a log file instead, this is accomplished by `fmtlog::setLogFile(filename)`. For performance, fmtlog internally buffer data, and under certain conditions will the buffer be flushed into the underlying file. The flush conditions are:
* The underlying FILE* is not managed by fmtlog, then fmtlog will not buffer at all. For example, the default stdout FILE* will not be buffered. User can also pass an existing FILE* and indicate whether fmtlog should manage it by `fmtlog::setLogFile(fp, manageFp)`, e.g. `fmtlog::setLogFile(stderr, false)`, then fmtlog will log into stderr without buffering.
* The buffer size is larger then 8 KB.
* The oldest data in the buffer has passed a specified duration. The duration is by default 3 seconds, and can be set by `fmtlog::setFlushDelay(ns)`.
* The new log has at least a specified flush log level. The default flush log level can't be reached by any log, but it can be set by `fmtlog::flushOn(logLevel)`.
* User can actively ask fmtlog to flush by `fmtlog::poll(true)`.

Optionally, user can ask fmtlog to close the log file by `fmtlog::closeLogFile()`, and subsequent log msgs will not be output.

In addition to writing to a FILE*, user can register a callback function to handle log msgs by `fmtlog::setLogCB(cb, minCBLogLevel)`. This can be useful in circumstances where warning/error msgs need to be published out in real time for alerting purposes. Log callback will not be buffered as log file, and can be triggered even when the file is closed.
The signiture of callback function is:
```c++
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
```

## Performance
Benchmark is done in terms of both average front-end latency and formatted throughput, with comparison to Nanolog and spdlog basic_logger_st. Test log messages use  [NanoLog benchmark Log-Messages-Map](https://github.com/PlatformLab/NanoLog#Log-Messages-Map), and header pattern uses spdlog default pattern(example: "[2021-05-04 10:36:38.098] [spdlog] [info] [bench.cc:111] "). Check [bench.cc](https://github.com/MengRao/fmtlog/blob/main/bench/bench.cc) for details, the results on a linux server with "Intel(R) Xeon(R) Gold 6144 CPU @ 3.50GHz" is:
| Message | fmtlog | Nanolog | spdlog |
|---------|:--------:|:--------:|:--------:|
|staticString |6.4 ns, 7.08 M/s|6.5 ns, 33.10 M/s|156.4 ns, 6.37 M/s|
|stringConcat |6.4 ns, 6.05 M/s|7.5 ns, 14.20 M/s|209.4 ns, 4.77 M/s|
|singleInteger |6.3 ns, 6.22 M/s|6.5 ns, 50.29 M/s|202.3 ns, 4.94 M/s|
|twoIntegers |6.4 ns, 4.87 M/s|6.6 ns, 39.25 M/s|257.2 ns, 3.89 M/s|
|singleDouble |6.2 ns, 5.37 M/s|6.5 ns, 39.62 M/s|225.0 ns, 4.44 M/s|
|complexFormat |6.4 ns, 2.95 M/s|6.7 ns, 24.30 M/s|390.9 ns, 2.56 M/s|

Note that the throughput of Nanolog is not comparable here because it outputs to binary log file instead of human-readable text format, for example, it saves an int64 timestamp instead of a long formatted date time string.

To be continued...
