### Verify ###################################################################

add_executable(VerifySeq
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

target_link_libraries(VerifySeq AnalysisUtil TinyAES)

### Quick Test ###############################################################

add_executable(test_quick
  src/test_quick.cpp
  )

format_output_name(test_quick "test_quick")

set_target_properties(test_quick PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

target_include_directories(test_quick
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_link_libraries(test_quick AnalysisUtil)
