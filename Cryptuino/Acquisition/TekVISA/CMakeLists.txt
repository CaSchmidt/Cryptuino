### VISA Library #############################################################

set(VISA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/TekVISA)

set(VISA_INCLUDES ${VISA_DIR}/include)

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
  set(VISA_LIBS ${VISA_DIR}/lib32/tkVisa32.lib)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(VISA_LIBS ${VISA_DIR}/lib64/tkVisa64.lib)
endif()

### Common Sources ###########################################################

list(APPEND common_HEADERS
  include/Instrument.h
  include/InstrUtil.h
  )

list(APPEND common_SOURCES
  src/Instrument.cpp
  src/InstrUtil.cpp
  )

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

### Grab #####################################################################

add_executable(GrabVISA
  ${common_HEADERS}
  ${common_SOURCES}
  src/main_grab.cpp
  )

format_output_name(GrabVISA "GrabVISA")

set_target_properties(GrabVISA PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

target_include_directories(GrabVISA
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/TekVISA/include
  )

target_link_libraries(GrabVISA CryptuinoUtil matio ${VISA_LIBS})

### Query ####################################################################

add_executable(QueryVISA
  src/main_query.cpp
  )

format_output_name(QueryVISA "QueryVISA")

target_include_directories(QueryVISA
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/TekVISA/include
  )

target_link_libraries(QueryVISA CryptuinoUtil ${VISA_LIBS})

### Sequence #################################################################

add_executable(SeqVISA
  src/main_seq.cpp
  )

format_output_name(SeqVISA "SeqVISA")

set_target_properties(SeqVISA PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

target_include_directories(SeqVISA
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/TekVISA/include
  )

target_link_libraries(SeqVISA TekVISA)
