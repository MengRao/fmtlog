# fmtlog
fmtlog is an asynchronous logging library using [fmt](https://github.com/fmtlib/fmt) library format. It's extremely performant with runtime latency lower than [NanoLog](https://github.com/PlatformLab/NanoLog) and throughput higher than [spdlog](https://github.com/gabime/spdlog).

## Features
* Very fast
* Headers only or compiled
* Feature rich formatting on top of excellent fmt library.
* Asynchronous logging with multithread support and can also be used synchronously.
* Custom formatting
* Custom handling - user can set a callback function to handle log msgs in addition to writing into file.
* Custom thread name
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

## To be continued
