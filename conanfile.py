import os
from conan import ConanFile
from conan.tools.files import copy
from conan.tools.cmake import CMakeToolchain, cmake_layout, CMake
from conan.tools.build import check_max_cppstd, check_min_cppstd, can_run


class SumConan(ConanFile):
    name = "fmtlog"
    version = "2.3.0"

    license = "MIT"
    author = "Meng Rao raomeng1@gmail.com"
    url = "https://github.com/MengRao/fmtlog"
    description = "fmtlog is a performant fmtlib-style logging library with latency in nanoseconds. "
    topics = ("logging", "logs", "log", "fmt", "fmtlib", "libfmt", "fmtlog")
    
    settings = "os", "arch", "compiler", "build_type"
    exports_sources = "CMakeLists.txt", "include/*", "test/*", "cmake/*"
    no_copy_source = True
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        self.test_requires("fmt/10.1.1")

    def validate(self):
        check_min_cppstd(self, "17")

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def test(self):
        if can_run(self):
            cmake = CMake(self)
            cmake.configure(variables={"FMTLOG_ENABLE_UNIT_TESTING":"ON"})
            cmake.build()
            cmd = "cd " + self.cpp.build.bindir + " && ctest . "
            self.run(cmd, env="conanrun")

    def package(self):
        # This will also copy the "include" folder
        copy(self, "*.h", self.source_folder, self.package_folder)

    def package_info(self):
        # For header-only packages, libdirs and bindirs are not used
        # so it's necessary to set those as empty.
        self.cpp_info.bindirs = []
        self.cpp_info.libdirs = []

    def package_id(self):
        self.info.clear()

