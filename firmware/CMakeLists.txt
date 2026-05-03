cmake_minimum_required(VERSION 3.22)

# ── Project ──────────────────────────────────────────────────────────────────
# Two build targets share this file:
#   arm-core-fw  →  cross-compiled ELF for the target MCU (STM32 or similar)
#   arm-core-sim →  native x86/x64 binary used for unit tests and HIL runs
#
# Select target via -DBUILD_TARGET=MCU or -DBUILD_TARGET=SIM (default: SIM)
# ─────────────────────────────────────────────────────────────────────────────

option(BUILD_TARGET "Build for MCU (cross) or SIM (native)" "SIM")

if(BUILD_TARGET STREQUAL "MCU")
    # Cross-compiler must be on PATH before CMake is invoked.
    # Typical: arm-none-eabi-g++ from arm-gnu-toolchain or xPack.
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR arm)
    set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_OBJCOPY       arm-none-eabi-objcopy)
    set(CMAKE_SIZE          arm-none-eabi-size)
endif()

project(arm-core VERSION 0.1 LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # used by clangd / VS Code intellisense

# ── Compiler flags ────────────────────────────────────────────────────────────
set(COMMON_FLAGS
    -Wall
    -Wextra
    -Wpedantic
    -Werror
    -fno-exceptions   # exceptions add code-size and latency on bare-metal
    -fno-rtti         # same reason
)

if(BUILD_TARGET STREQUAL "MCU")
    # Cortex-M4F with FPU — adjust to your exact MCU
    list(APPEND COMMON_FLAGS
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -Os               # optimise for size; change to -O2 for speed-critical loops
        -g                # keep debug info for OpenOCD / J-Link
        -ffunction-sections
        -fdata-sections
    )
    set(LINK_FLAGS "-Wl,--gc-sections -specs=nano.specs -specs=nosys.specs")
else()
    list(APPEND COMMON_FLAGS -O2 -g -fsanitize=address,undefined)
    set(LINK_FLAGS "-fsanitize=address,undefined")
endif()

add_compile_options(${COMMON_FLAGS})

# ── Include paths ─────────────────────────────────────────────────────────────
include_directories(include)

# ── Source files ──────────────────────────────────────────────────────────────
set(DRIVER_SOURCES
    drivers/can_driver.cpp
    drivers/spi_driver.cpp
    drivers/i2c_driver.cpp
)

set(APP_SOURCES
    src/main.cpp
)

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(arm-core ${APP_SOURCES} ${DRIVER_SOURCES})
target_link_options(arm-core PRIVATE ${LINK_FLAGS})

if(BUILD_TARGET STREQUAL "MCU")
    # Post-build: print memory usage and produce a raw binary alongside the ELF
    add_custom_command(TARGET arm-core POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:arm-core>
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:arm-core>
                $<TARGET_FILE_DIR:arm-core>/arm-core.bin
        COMMENT "Generating arm-core.bin"
    )
endif()

# ── Tests (native only) ───────────────────────────────────────────────────────
if(NOT BUILD_TARGET STREQUAL "MCU")
    enable_testing()
    add_subdirectory(tests)
endif()
