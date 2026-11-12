# SPDX-License-Identifier: MIT
#
# hiwonder-servo-classic-sdk
#   High-performance Linux C++17 SDK + pybind11 Python wrapper for the
#   Hiwonder / LewanSoul Classic Serial Bus Servo family (0x55 0x55 protocol).
#
cmake_minimum_required(VERSION 3.16)

project(hiwonder_servo_classic
  VERSION 0.1.0
  DESCRIPTION "Linux C++17 SDK and pybind11 wrapper for Hiwonder/LewanSoul classic bus servos"
  HOMEPAGE_URL "https://github.com/hiwonder-servo-classic/hiwonder-servo-classic-sdk"
  LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(BUILD_PYTHON_BINDINGS "Build the pybind11 Python extension module" ON)
option(BUILD_EXAMPLES        "Build the C++ example programs"             ON)
option(BUILD_TESTS           "Build the unit tests and register them with CTest" ON)
option(HIWONDER_SERVO_CLASSIC_WARNINGS "Enable -Wall -Wextra on our own targets" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Release/Debug)" FORCE)
endif()

# Default to a static lib; honour the conventional override for shared builds.
# Wheels link the SDK statically even if BUILD_SHARED_LIBS is set: only the
# extension is installed in a wheel, so no external SDK .so may be required.
if(BUILD_SHARED_LIBS AND NOT SKBUILD)
  set(HIWONDER_SERVO_CLASSIC_LIBRARY_KIND SHARED)
else()
  set(HIWONDER_SERVO_CLASSIC_LIBRARY_KIND STATIC)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX "" CACHE STRING "")
endif()

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
set(HIWONDER_SERVO_CLASSIC_SOURCES
  src/serial_bus.cpp
  src/servo.cpp)

set(HIWONDER_SERVO_CLASSIC_HEADERS
  include/HiwonderBus/HiwonderBus.hpp
  include/HiwonderBus/exceptions.hpp
  include/HiwonderBus/protocol.hpp
  include/HiwonderBus/serial_bus.hpp
  include/HiwonderBus/servo.hpp)

add_library(hiwonder_servo_classic ${HIWONDER_SERVO_CLASSIC_LIBRARY_KIND}
  ${HIWONDER_SERVO_CLASSIC_SOURCES}
  ${HIWONDER_SERVO_CLASSIC_HEADERS})
add_library(HiwonderBus::hiwonder_servo_classic ALIAS hiwonder_servo_classic)

target_compile_features(hiwonder_servo_classic PUBLIC cxx_std_17)

target_include_directories(hiwonder_servo_classic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Zero runtime dependencies beyond libc/libstdc++ and the POSIX headers.
if(HIWONDER_SERVO_CLASSIC_WARNINGS)
  target_compile_options(hiwonder_servo_classic PRIVATE
    -Wall -Wextra -Wshadow -Wcast-qual -Wconversion -Wno-unused-parameter)
endif()

set_target_properties(hiwonder_servo_classic PROPERTIES
  OUTPUT_NAME hiwonder_servo_classic
  EXPORT_NAME hiwonder_servo_classic
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ON)

# Threads are used by SerialBus's internal mutex.
find_package(Threads QUIET)
if(Threads_FOUND)
  target_link_libraries(hiwonder_servo_classic PUBLIC Threads::Threads)
endif()

# ---------------------------------------------------------------------------
# Python bindings (pybind11)
# ---------------------------------------------------------------------------
if(BUILD_PYTHON_BINDINGS)
  # Inside a scikit-build-core wheel build these are mandatory; for a plain
  # `cmake --build` they are opportunistic so the C++ library never fails to
  # configure just because Python dev headers are missing.
  if(DEFINED SKBUILD)
    set(_hsc_python_required REQUIRED)
  else()
    set(_hsc_python_required QUIET)
  endif()

  find_package(Python3 COMPONENTS Interpreter Development.Module ${_hsc_python_required})

  if(NOT TARGET pybind11::module)
    # Ask the interpreter where its pybind11 lives (works for venvs, distro
    # packages and pip installs alike).
    find_program(CMAKE_PROGRAM_PYTHON3 NAMES python3 python)
    if(CMAKE_PROGRAM_PYTHON3)
      execute_process(
        COMMAND "${CMAKE_PROGRAM_PYTHON3}" -c "import pybind11,sys;sys.stdout.write(pybind11.get_cmake_dir())"
        OUTPUT_VARIABLE _hsc_pybind11_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
      if(_hsc_pybind11_dir)
        list(APPEND CMAKE_PREFIX_PATH "${_hsc_pybind11_dir}")
      endif()
    endif()
    find_package(pybind11 CONFIG ${_hsc_python_required})
  endif()

  if(TARGET pybind11::module AND Python3_FOUND)
    message(STATUS "hiwonder_servo_classic: building Python module _hiwonder_bus")

    pybind11_add_module(_hiwonder_bus python/bindings.cpp)
    target_link_libraries(_hiwonder_bus PRIVATE hiwonder_servo_classic)
    target_compile_features(_hiwonder_bus PRIVATE cxx_std_17)

    # Ship the extension inside the hiwonder_bus package directory so both
    # `python/hiwonder_bus/_hiwonder_bus.so` (in-tree) and the installed wheel
    # layout agree.
    set_target_properties(_hiwonder_bus PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/hiwonder_bus"
      LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/python/hiwonder_bus"
      LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/python/hiwonder_bus")

    # scikit-build-core rewrites install destinations to sit at the wheel
    # root, so the extension lands in <site-packages>/hiwonder_bus/.
    if(SKBUILD)
      install(TARGETS _hiwonder_bus
        LIBRARY DESTINATION hiwonder_bus
        COMPONENT python)
    endif()
  else()
    message(STATUS "hiwonder_servo_classic: pybind11 or Python3 dev not found; "
                   "skipping the Python extension (BUILD_PYTHON_BINDINGS=OFF silences this)")
  endif()
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------
if(BUILD_EXAMPLES)
  foreach(example scan_bus sync_biped_demo)
    add_executable(${example} examples/cpp/${example}.cpp)
    target_link_libraries(${example} PRIVATE hiwonder_servo_classic)
    target_compile_features(${example} PRIVATE cxx_std_17)
    if(HIWONDER_SERVO_CLASSIC_WARNINGS)
      target_compile_options(${example} PRIVATE -Wall -Wextra)
    endif()
  endforeach()
  if(NOT SKBUILD)
    install(TARGETS scan_bus sync_biped_demo
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/hiwonder_servo_classic/examples)
  endif()
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTS)
  enable_testing()
  add_executable(test_protocol tests/test_protocol.cpp)
  target_link_libraries(test_protocol PRIVATE hiwonder_servo_classic)
  target_compile_features(test_protocol PRIVATE cxx_std_17)
  if(HIWONDER_SERVO_CLASSIC_WARNINGS)
    target_compile_options(test_protocol PRIVATE -Wall -Wextra)
  endif()
  # tests/fake_transport.hpp is header-only and lives next to the test.
  target_include_directories(test_protocol PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  add_test(NAME protocol_tests COMMAND test_protocol)
  add_executable(test_review_regressions tests/test_review_regressions.cpp)
  target_link_libraries(test_review_regressions PRIVATE hiwonder_servo_classic)
  add_test(NAME review_regressions COMMAND test_review_regressions)
  set_tests_properties(review_regressions PROPERTIES TIMEOUT 10)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(test_posix_transport tests/test_posix_transport.cpp)
    target_link_libraries(test_posix_transport PRIVATE hiwonder_servo_classic)
    add_test(NAME posix_transport_tests COMMAND test_posix_transport)
    set_tests_properties(posix_transport_tests PROPERTIES TIMEOUT 10)
  endif()
  set_tests_properties(protocol_tests PROPERTIES
    PASS_REGULAR_EXPRESSION "All tests passed")

  # Each public header must be includable on its own: a missing `#include` in a
  # header is invisible inside this project (some other header saves you) and
  # only bites downstream users. Compile a tiny TU per header to catch it.
  foreach(header
      protocol exceptions serial_bus servo HiwonderBus testing/simulator)
    get_filename_component(basename ${header} NAME)
    set(generated "${CMAKE_CURRENT_BINARY_DIR}/header_check_${basename}.cpp")
    file(WRITE "${generated}"
      "#include <HiwonderBus/${header}.hpp>\n"
      "int main() { return 0; }\n")
    add_executable(header_check_${basename} ${generated})
    target_link_libraries(header_check_${basename} PRIVATE hiwonder_servo_classic)
    target_compile_features(header_check_${basename} PRIVATE cxx_std_17)
    add_dependencies(header_check_${basename} hiwonder_servo_classic)
    add_test(NAME header_check_${basename} COMMAND header_check_${basename})
  endforeach()
endif()

# ---------------------------------------------------------------------------
# Install & export (find_package(hiwonder_servo_classic REQUIRED))
#
# These rules exist for C++ consumers. A wheel build (SKBUILD) only needs the
# Python extension, so the headers, static library and CMake package are left
# out of the wheel: they would be dead weight for pip users.
# ---------------------------------------------------------------------------
if(NOT SKBUILD)
  install(TARGETS hiwonder_servo_classic
    EXPORT hiwonder_servo_classicTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/HiwonderBus
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp")

  install(EXPORT hiwonder_servo_classicTargets
    FILE hiwonder_servo_classicTargets.cmake
    NAMESPACE HiwonderBus::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hiwonder_servo_classic)

  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/hiwonder_servo_classicConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/hiwonder_servo_classicConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hiwonder_servo_classic
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR)

  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/hiwonder_servo_classicConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hiwonder_servo_classicConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/hiwonder_servo_classicConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hiwonder_servo_classic)
endif()

# ---------------------------------------------------------------------------
# Config summary
# ---------------------------------------------------------------------------
if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  return()  # consumed via add_subdirectory(): leave the summary to the parent
endif()

message(STATUS "")
message(STATUS "hiwonder_servo_classic ${PROJECT_VERSION}")
message(STATUS "  library kind        : ${HIWONDER_SERVO_CLASSIC_LIBRARY_KIND}")
message(STATUS "  python bindings     : ${BUILD_PYTHON_BINDINGS}")
message(STATUS "  examples            : ${BUILD_EXAMPLES}")
message(STATUS "  tests               : ${BUILD_TESTS}")
message(STATUS "  install prefix      : ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
