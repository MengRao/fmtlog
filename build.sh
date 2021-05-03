BUILD_DIR=${BUILD_DIR:-.build}

mkdir -p "$BUILD_DIR" \
  && cd "$BUILD_DIR" \
  && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..\
  && cmake --build . -j \
  && cmake --install . --prefix .
