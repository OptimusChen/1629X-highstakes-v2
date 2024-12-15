cmake_minimum_required(VERSION 3.10)
project(project-name)

set(CMAKE_CXX_STANDARD 20)

enable_testing()

# Disable deprecated copy warning
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-deprecated-copy)
endif()

# Add GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.10.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include_directories(include/)
# file(GLOB LIB_SOURCES "src/lib/*.cpp")
file(GLOB MOCK_PROS_SOURCES "tests/mockpros/*.cpp")

add_executable(
  main_test
  tests/main_test.cpp
  ${LIB_SOURCES}
  ${MOCK_PROS_SOURCES}
)
target_link_libraries(
  main_test
  gtest_main
)

add_executable(
  ParticleFilterTest
  tests/ParticleFilter_test.cpp
  ${LIB_SOURCES}
  ${MOCK_PROS_SOURCES}
)
target_link_libraries(
  ParticleFilterTest
  gtest_main
)

add_executable(
  MotionProfilingTest
  tests/motionProfiling_test.cpp
  src/lib/motionProfiling.cpp
  ${LIB_SOURCES}
  ${MOCK_PROS_SOURCES}
)
target_link_libraries(
    MotionProfilingTest
  gtest_main
)

include(GoogleTest)
gtest_discover_tests(ParticleFilterTest MotionProfilingTest)