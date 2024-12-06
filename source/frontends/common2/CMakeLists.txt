include(GNUInstallDirs)

set(SOURCE_FILES
  commonframe.cpp
  commoncontext.cpp
  controllerdoublepress.cpp
  utils.cpp
  timer.cpp
  speed.cpp
  )

set(HEADER_FILES
  commonframe.h
  commoncontext.h
  controllerdoublepress.h
  utils.h
  timer.h
  speed.h
  )

if (BUILD_APPLEN OR BUILD_QAPPLE OR BUILD_SA2)
  set(SOURCE_FILES ${SOURCE_FILES}
    fileregistry.cpp
    gnuframe.cpp
    programoptions.cpp
    ptreeregistry.cpp
  )
  set(HEADER_FILES ${HEADER_FILES}
    fileregistry.h
    gnuframe.h
    programoptions.h
    ptreeregistry.h
  )
endif()

add_library(common2 STATIC
  ${SOURCE_FILES}
  ${HEADER_FILES}
  )

target_compile_features(common2 PUBLIC cxx_std_17)

target_include_directories(common2 PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  )

target_link_libraries(common2 PRIVATE
  appleii
)

if (BUILD_APPLEN OR BUILD_QAPPLE OR BUILD_SA2)
  # boost is needed by programoptions and ptreeregistry, which aren't used
  # by libretro. need better separation of libretro common and linux common
  find_package(Boost REQUIRED
    COMPONENTS program_options
    )

  target_include_directories(common2 PRIVATE
    ${Boost_INCLUDE_DIRS}
    )

  target_link_libraries(common2 PRIVATE
    Boost::program_options
  )
endif()

if (NOT WIN32)
  target_link_libraries(common2 PRIVATE
    windows
  )
endif()

file(RELATIVE_PATH ROOT_PATH ${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR})
if ("${ROOT_PATH}" STREQUAL "")
  # if the 2 paths are the same
  set(ROOT_PATH "./")
endif()
file(RELATIVE_PATH SHARE_PATH ${CMAKE_INSTALL_FULL_BINDIR} ${CMAKE_INSTALL_FULL_DATADIR}/applewin)

configure_file(config.h.in config.h)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/resource
  DESTINATION ${CMAKE_INSTALL_DATADIR}/applewin)
