#include "lib.h"
#include "../fmtlog.h"

int main(int argc, char** argv) {
  logi("link test: {}", 123);
  libFun(321);
  return 0;
}
