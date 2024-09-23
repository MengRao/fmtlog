include(${CMAKE_CURRENT_LIST_DIR}/../getCPM.cmake)

set(fmt_VERSION 11.0.2)
CPMAddPackage(
  NAME fmt
  # GIT_TAG ${fmt_VERSION}
  # GITHUB_REPOSITORY fmtlib/fmt
  VERSION ${fmt_VERSION}
  URL "https://github.com/fmtlib/fmt/releases/download/${fmt_VERSION}/fmt-${fmt_VERSION}.zip"
  OPTIONS "FMT_OS OFF" "fmtlog_ENABLE_CPM=1"
)
