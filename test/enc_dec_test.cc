#include "../fmtlog.h"
#include "fmt/ranges.h"
#include <assert.h>
#include <iostream>
using namespace std;
using namespace fmt::literals;

struct MyType
{
  MyType(int val)
    : v(val) {}
  ~MyType() {
    dtor_cnt++;
    // fmt::print("dtor_cnt: {}\n", dtor_cnt);
  }
  int v;
  static int dtor_cnt;
};

int MyType::dtor_cnt = 0;

template<>
struct fmt::formatter<MyType> : formatter<int>
{
  // parse is inherited from formatter<string_view>.
  template<typename FormatContext>
  auto format(const MyType& val, FormatContext& ctx) const {
    return formatter<int>::format(val.v, ctx);
  }
};

struct MovableType
{
public:
  MovableType(int v = 0)
    : val{MyType(v)} {}

  std::vector<MyType> val;
};

template<>
struct fmt::formatter<MovableType> : formatter<int>
{
  // parse is inherited from formatter<string_view>.
  template<typename FormatContext>
  auto format(const MovableType& val, FormatContext& ctx) {
    return formatter<int>::format(val.val[0].v, ctx);
  }
};

template<typename S, typename... Args>
void test(const S& format, Args&&... args) {
  fmt::detail::check_format_string<Args...>(format);
  auto sv = fmt::string_view(format);
  size_t formatted_size = fmt::formatted_size(fmt::runtime(sv), std::forward<Args>(args)...);
  string ans = fmt::format(fmt::runtime(sv), std::forward<Args>(args)...);
  assert(ans.size() == formatted_size);

  auto unnamed_format = fmtlog::unNameFormat<false>(sv, nullptr, args...);
  fmt::print("unnamed_format: {}\n", unnamed_format);
  size_t cstringSizes[1000];
  char buf[1024];
  int allocSize = fmtlog::getArgSizes<0>(cstringSizes, args...);
  const char* ret = fmtlog::encodeArgs<0>(cstringSizes, buf, std::forward<Args>(args)...);
  // fmt::print("=========\n");
  assert(ret - buf == allocSize);
  fmtlog::MemoryBuffer buffer;
  int argIdx = -1;
  std::vector<fmt::basic_format_arg<fmtlog::Context>> format_args;

  ret = fmtlog::formatTo<Args...>(unnamed_format, buf, buffer, argIdx, format_args);
  assert(ret - buf == allocSize);

  string_view res(buffer.data(), buffer.size());
  fmt::print("res: {}\n", res);
  fmt::print("ans: {}\n", ans);
  assert(res == ans);
}

int main() {
  char cstring[100] = "cstring cstring";
  const char* p = "haha";
  const char* pcstring = cstring;
  string str("str");
  char ch = 'f';
  char& ch2 = ch;
  int i = 5;
  int& ri = i;
  double d = 3.45;
  float f = 55.2;
  uint16_t short_int = 2222;

  test("test basic types: {}, {}, {}, {}, {}, {}, {}, {}, {:.1f}, {}, {}, {}, {}, {}, {}, {}", cstring, p, pcstring,
       "wow", 'a', 5, str, string_view(str), 1.34, ch, ch2, i, ri, d, f, short_int);

  test("test positional, {one}, {two:>5}, {three}, {four}, {0:.1f}", 5.012, "three"_a = 3, "two"_a = "two",
       "one"_a = string("one"), "four"_a = string("4"));
  test("test dynamic spec: {:.{}f}, {:*^30}", 3.14, 1, "centered");
  test("test positional spec: int: {0:d};  hex: {0:#x};  oct: {0:#o};  bin: {0:#b}", 42);

  test("test custom types: {}, {}, {}", MyType(1), MyType(2), MovableType(3));

  test("test ranges: {}, {}", vector<int>{1, 2, 3}, vector<MyType>{4, 5, 6});

  int dtor_cnt;
  {
    MovableType val(123);
    dtor_cnt = MyType::dtor_cnt;
    test("test copy: {}", val);
    assert(MyType::dtor_cnt == dtor_cnt + 1);
    assert(val.val.size() == 1);
    dtor_cnt = MyType::dtor_cnt;
  }
  assert(MyType::dtor_cnt == dtor_cnt + 1);
  {
    MovableType val(456);
    dtor_cnt = MyType::dtor_cnt;
    test("test move: {}", std::move(val));
    assert(MyType::dtor_cnt == dtor_cnt + 1);
    assert(val.val.size() == 0);
    dtor_cnt = MyType::dtor_cnt;
  }
  assert(MyType::dtor_cnt == dtor_cnt);

  fmt::print("tests passed\n");

  return 0;
}

