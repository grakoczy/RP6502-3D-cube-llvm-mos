cmake_minimum_required(VERSION 3.18)
add_subdirectory(tools)
set(LLVM_MOS_PLATFORM rp6502)
find_package(llvm-mos-sdk REQUIRED)
project(MY-RP6502-PROJECT)
add_executable(cube)
rp6502_executable(cube)
target_sources(cube PRIVATE
    src/colors.c
    src/bitmap_graphics_db.c
    src/main.c
)
