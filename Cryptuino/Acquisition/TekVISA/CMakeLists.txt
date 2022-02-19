### VISA Library #############################################################

set(VISA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/TekVISA)

set(VISA_INCLUDES ${VISA_DIR}/include)

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
  set(VISA_LIBS ${VISA_DIR}/lib32/tkVisa32.lib)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(VISA_LIBS ${VISA_DIR}/lib64/tkVisa64.lib)
endif()

### Library ##################################################################

list(APPEND TekVISA_HEADERS
  include/Instrument.h
  include/TekVISA.h
  )

list(APPEND TekVISA_SOURCES
  src/Instrument.cpp
  src/TekVISA.cpp
  )

add_library(TekVISA STATIC
  ${TekVISA_HEADERS}
  ${TekVISA_SOURCES}
  )

format_output_name(TekVISA "TekVISA")

target_include_directories(TekVISA
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${VISA_INCLUDES}
  )

set_target_properties(TekVISA PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

target_link_libraries(TekVISA
  PUBLIC  AcquisitionUtil
  PRIVATE ${VISA_LIBS}
  )

### Sequence #################################################################

add_executable(SeqVISA
  src/main_seq.cpp
  )

format_output_name(SeqVISA "SeqVISA")

set_target_properties(SeqVISA PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

if(MSVC)
  target_compile_definitions(SeqVISA
    PRIVATE _CRT_SECURE_NO_WARNINGS
    )
endif()

target_include_directories(SeqVISA
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/TekVISA/include
  )

target_link_libraries(SeqVISA TekVISA)
