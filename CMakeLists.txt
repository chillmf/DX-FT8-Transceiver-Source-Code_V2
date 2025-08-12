cmake_minimum_required(VERSION 3.16)

include(arm-gcc.cmake)

project(Katy LANGUAGES C CPP ASM)

# -----------------------------------------------------------------------------
# Toolchain (normally supplied via -DCMAKE_TOOLCHAIN_FILE)
# -----------------------------------------------------------------------------
if(NOT CMAKE_C_COMPILER)
  set(CMAKE_SYSTEM_NAME Generic)
  set(CMAKE_C_COMPILER  arm-none-eabi-gcc)
  set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
  set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
  set(CMAKE_SIZE         arm-none-eabi-size)
endif()

# -----------------------------------------------------------------------------
# Compiler / assembler flags (mirror original build.sh)
# -----------------------------------------------------------------------------
set(CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")
string(REPLACE " " ";" CPU_FLAGS_LIST "${CPU_FLAGS}")

set(COMMON_DEFS "-DUSE_HAL_DRIVER -DSTM32F746xx -DUSE_STM32746G_DISCO -DUSE_IOEXPANDER")
set(WARNING_FLAGS "-Wall -Wextra")
set(OPT_FLAGS "-Os -ffunction-sections -fdata-sections")

set(CMAKE_C_FLAGS   "${CPU_FLAGS} ${WARNING_FLAGS} -std=gnu11 -g3 ${COMMON_DEFS} ${OPT_FLAGS}")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} ${WARNING_FLAGS} -x assembler-with-cpp ${OPT_FLAGS}")

# -----------------------------------------------------------------------------
# Include directories (both underscore & hyphen variants just in case)
# -----------------------------------------------------------------------------
include_directories(
  ${CMAKE_SOURCE_DIR}/Inc
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc
  ${CMAKE_SOURCE_DIR}/Drivers/BSP/STM32746G_DISCOVERY
  ${CMAKE_SOURCE_DIR}/Drivers/BSP/STM32746G-Discovery
  ${CMAKE_SOURCE_DIR}/Drivers/BSP/Common
  ${CMAKE_SOURCE_DIR}/DSP_CMSIS
  ${CMAKE_SOURCE_DIR}/FT8_library
  ${CMAKE_SOURCE_DIR}/Middlewares/src
  ${CMAKE_SOURCE_DIR}/Utilities
  ${CMAKE_SOURCE_DIR}/Utilities/Fonts
)

# Components headers (e.g. wm8994.h)
file(GLOB COMPONENT_DIRS LIST_DIRECTORIES true ${CMAKE_SOURCE_DIR}/Drivers/BSP/*)
foreach(COMP_DIR ${COMPONENT_DIRS})
  if(IS_DIRECTORY ${COMP_DIR})
    include_directories(${COMP_DIR})
  endif()
endforeach()

# -----------------------------------------------------------------------------
# Source files (replicate build.sh behaviour)
# -----------------------------------------------------------------------------
file(GLOB_RECURSE HAL_SOURCES Drivers/STM32F7xx_HAL_Driver/Src/*.c)

file(GLOB_RECURSE BSP_BOARD_SOURCES
     Drivers/BSP/STM32746G_DISCOVERY/*.c)
file(GLOB_RECURSE BSP_COMP_SOURCES Drivers/BSP/*.c)

file(GLOB DSP_C_SOURCES DSP_CMSIS/*.c)
file(GLOB DSP_ASM_SOURCES DSP_CMSIS/*.s)
file(GLOB FT8_SOURCES FT8_library/*.c)
file(GLOB_RECURSE MIDDLEWARES_SOURCES Middlewares/src/*.c)
file(GLOB SRC_SOURCES Src/*.c)
file(GLOB SRCPP_SOURCES Src/*.cpp)
file(GLOB SYSTEM_SOURCES Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/*.c)
set(STARTUP_FILE Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f746xx.s)

add_executable(Katy.elf
  ${STARTUP_FILE}
  ${SYSTEM_SOURCES}
  ${HAL_SOURCES}
  ${BSP_BOARD_SOURCES}
  ${BSP_COMP_SOURCES}
  ${DSP_C_SOURCES}
  ${DSP_ASM_SOURCES}
  ${FT8_SOURCES}
  ${MIDDLEWARES_SOURCES}
  ${SRC_SOURCES}
  ${SRCPP_SOURCES}
)

# --- Compile Src/*.c with -O0 to make debugging easier
set_source_files_properties(
    Src/main.cpp
    # ${SRC_SOURCES}
    # ${SRCPP_SOURCES}
    PROPERTIES
        COMPILE_OPTIONS "-O0"    # or COMPILE_FLAGS "-O0" on <3.3
)

# --- Treat warnings as errors for SRC_SOURCES
set_source_files_properties(
    ${SRC_SOURCES} ${SRCPP_SOURCES}
    PROPERTIES
        COMPILE_OPTIONS "-Werror"
)

# -----------------------------------------------------------------------------
# Linker script & options (mirror build.sh)
# -----------------------------------------------------------------------------
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F746NGHx_FLASH.ld)

target_link_options(Katy.elf PRIVATE
  ${CPU_FLAGS_LIST}
  -T${LINKER_SCRIPT}
  --specs=nosys.specs
  --specs=nano.specs
  -static
  -Wl,--gc-sections
  -Wl,-Map=Katy.map
  -Wl,--start-group -lc -lm -Wl,--end-group
)

# -----------------------------------------------------------------------------
# Post‑build helpers (same as build.sh plus size output)
# -----------------------------------------------------------------------------
add_custom_command(TARGET Katy.elf POST_BUILD
  COMMAND ${CMAKE_SIZE} Katy.elf
  COMMAND ${CMAKE_OBJCOPY} -O binary Katy.elf Katy.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex Katy.elf Katy.hex
  COMMAND arm-none-eabi-objdump -h -S Katy.elf > Katy.list
)
