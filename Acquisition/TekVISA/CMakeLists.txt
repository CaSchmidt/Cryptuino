cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

### Project ##################################################################

project(TekVISA)

# Custom CMake utilities
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(FormatOutputName)

### 3rd Party Dependencies ###################################################

add_subdirectory(../../3rdparty/zlib1211
  ${CMAKE_CURRENT_BINARY_DIR}/zlib1211
  )

set(MATIO_MAT73  OFF CACHE BOOL "" FORCE)
set(MATIO_SHARED OFF CACHE BOOL "" FORCE)

add_subdirectory(../../3rdparty/matio1521
  ${CMAKE_CURRENT_BINARY_DIR}/mat1521
  )

format_output_name(matio "matio")

add_subdirectory(../../csUtil/csUtil
  ${CMAKE_CURRENT_BINARY_DIR}/csUtil
  )

### VISA Library #############################################################

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
  list(APPEND VISA_LIBS
    ${TekVISA_SOURCE_DIR}/3rdparty/TekVISA/lib32/tkVisa32.lib
    )
elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
  list(APPEND VISA_LIBS
    ${TekVISA_SOURCE_DIR}/3rdparty/TekVISA/lib64/tkVisa64.lib
    )
endif()

### Common Sources ###########################################################

list(APPEND common_HEADERS
  include/buffer.h
  include/CmdIntegralValueOption.h
  include/CmdOption.h
  include/CmdOptions.h
  include/CmdStringValueOption.h
  include/instrument.h
  include/util.h
  )

list(APPEND common_SOURCES
  src/CmdOption.cpp
  src/CmdOptions.cpp
  src/CmdStringValueOption.cpp
  src/instrument.cpp
  src/util.cpp
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
  PRIVATE ${TekVISA_SOURCE_DIR}/3rdparty/TekVISA/include
  )

target_link_libraries(GrabVISA csUtil matio ${VISA_LIBS})

### Query ####################################################################

add_executable(QueryVISA
  src/main_query.cpp
  )

format_output_name(QueryVISA "QueryVISA")

target_include_directories(QueryVISA
  PRIVATE ${TekVISA_SOURCE_DIR}/3rdparty/TekVISA/include
  )

target_link_libraries(QueryVISA ${VISA_LIBS})

### Sequence #################################################################

add_executable(SeqVISA
  ${common_HEADERS}
  ${common_SOURCES}
  src/main_seq.cpp
  )

format_output_name(SeqVISA "SeqVISA")

set_target_properties(SeqVISA PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

target_include_directories(SeqVISA
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${TekVISA_SOURCE_DIR}/3rdparty/TekVISA/include
  )

target_link_libraries(SeqVISA csUtil matio ${VISA_LIBS})
