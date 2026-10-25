add_library(spng STATIC
  spng.c
  )

target_include_directories(spng PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  )
