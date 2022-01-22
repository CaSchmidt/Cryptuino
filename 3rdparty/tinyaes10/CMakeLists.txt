list(APPEND TinyAES_HEADERS
  include/aes.h
  include/aes.hpp
  )

list(APPEND TinyAES_SOURCES
  src/aes.c
  )

add_library(TinyAES STATIC
  ${TinyAES_HEADERS}
  ${TinyAES_SOURCES}
  )

format_output_name(TinyAES "TinyAES")

target_include_directories(TinyAES
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_link_libraries(TinyAES)
