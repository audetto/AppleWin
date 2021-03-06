cmake_minimum_required(VERSION 3.9)

include(CPack)

project(applewin)

add_compile_options(-Werror=return-type)

MESSAGE("CMAKE_BUILD_TYPE:                ${CMAKE_BUILD_TYPE}")
MESSAGE("CMAKE_CXX_FLAGS:                 ${CMAKE_CXX_FLAGS}")
MESSAGE("CMAKE_CXX_FLAGS_RELEASE:         ${CMAKE_CXX_FLAGS_RELEASE}")
MESSAGE("CMAKE_CXX_FLAGS_DEBUG:           ${CMAKE_CXX_FLAGS_DEBUG}")
MESSAGE("CMAKE_CXX_FLAGS_RELWITHDEBINFO:  ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

# this only affects common2, the others are already build with fPIC by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

execute_process(COMMAND uname -n
  OUTPUT_VARIABLE UNAME
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(${UNAME} STREQUAL raspberrypi)
  # it is too slow and might cause out of memory issues
  # more forensic is required
  MESSAGE("Raspberry Pi detected: IPO disabled")
else()
  include(CheckIPOSupported)
  check_ipo_supported()
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION FALSE)
endif()

include_directories(source)

add_subdirectory(source)
add_subdirectory(source/frontends/common2)
add_subdirectory(source/frontends/ncurses)
add_subdirectory(source/frontends/qt)
add_subdirectory(source/frontends/sdl)
add_subdirectory(source/frontends/libretro)
add_subdirectory(test/TestCPU6502)
