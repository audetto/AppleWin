include(FindPkgConfig)

add_library(appleii SHARED
  SaveState.cpp
  Pravets.cpp
  Tape.cpp
  YamlHelper.cpp
  Log.cpp
  Disk.cpp
  DiskFormatTrack.cpp
  DiskImage.cpp
  DiskImageHelper.cpp
  Harddisk.cpp
  Memory.cpp
  CPU.cpp
  6821.cpp
  NoSlotClock.cpp
  SAM.cpp
  z80emu.cpp
  ParallelPrinter.cpp
  MouseInterface.cpp
  LanguageCard.cpp
  RGBMonitor.cpp

  linux/data.cpp
  linux/dummies.cpp
  linux/wwrapper.cpp
  linux/state.cpp
  linux/benchmark.cpp
  linux/paddle.cpp
  linux/version.cpp
  linux/stringcb.cpp
  linux/registry.cpp

  Z80VICE/z80.cpp
  Z80VICE/z80mem.cpp
  Z80VICE/daa.cpp
  )

pkg_check_modules(YAML REQUIRED yaml-0.1)
pkg_check_modules(MINIZIP REQUIRED minizip)
find_package(Boost REQUIRED)

target_include_directories(appleii PRIVATE
  ${YAML_INCLUDE_DIRS}
  ${MINIZIP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

target_link_libraries(appleii
  ${YAML_LIBRARIES}
  ${MINIZIP_LIBRARIES}
  )