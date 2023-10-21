set(sources
)

set(exe_sources
		${sources}
)

set(headers
    include/fmtlog/fmtlog.h
    include/fmtlog/fmtlog-inl.h
)

set(test_sources
  src/enc_dec_test.cc
  src/log_test.cc
  src/multithread_test.cc
)
