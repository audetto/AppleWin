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

target_link_libraries(appleii
  yaml
  z
  minizip
  )