add_library(spng STATIC
  spng.c
  )

target_compile_definitions(spng PUBLIC
  SPNG_STATIC
  )

target_include_directories(spng PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  )
