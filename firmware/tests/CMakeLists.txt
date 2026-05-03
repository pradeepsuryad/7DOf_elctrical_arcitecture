# Unit tests — compiled and run on the host (no MCU needed).
#
# We use Google Test (fetched automatically via CMake FetchContent).
# Run with:  cmake --build build --target test
#        or: ctest --test-dir build -V

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
FetchContent_MakeAvailable(googletest)

# ── Fake (SIM) implementations of the HAL interfaces ─────────────────────────
# These let you test motor-control logic without real hardware.
# Add test_*.cpp files here as control-loop code grows.

add_executable(arm-core-tests
    test_joint_controller.cpp
    # Sources under test (SIM build — no STM32 HAL calls compiled in)
    ${CMAKE_SOURCE_DIR}/src/joint_controller.cpp
)

target_link_libraries(arm-core-tests PRIVATE GTest::gtest_main)
target_include_directories(arm-core-tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}   # for fake_can_bus.hpp
)

include(GoogleTest)
gtest_discover_tests(arm-core-tests)
