### Verify ###################################################################

add_executable(VerifySeq
  include/Campaign.h
  src/Campaign.cpp
  src/main_verify.cpp
  )

format_output_name(VerifySeq "VerifySeq")

set_target_properties(VerifySeq PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

target_include_directories(VerifySeq
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_link_libraries(VerifySeq CryptuinoUtil TinyAES)
