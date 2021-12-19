cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

### Project ##################################################################

project(TekVISA)

# Custom CMake utilities
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(FormatOutputName)

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

### Grab #####################################################################

add_executable(GrabVISA
  src/main_grab.cpp
  )

format_output_name(GrabVISA "GrabVISA")

set_property(TARGET GrabVISA
  PROPERTY CXX_STANDARD 17
  )

set_property(TARGET GrabVISA
  PROPERTY CXX_STANDARD_REQUIRED ON
  )

target_include_directories(GrabVISA
  PRIVATE ${TekVISA_SOURCE_DIR}/3rdparty/TekVISA/include
  )

target_link_libraries(GrabVISA ${VISA_LIBS})

### Query ####################################################################

add_executable(QueryVISA
  src/main_query.cpp
  )

format_output_name(QueryVISA "QueryVISA")

target_include_directories(QueryVISA
  PRIVATE ${TekVISA_SOURCE_DIR}/3rdparty/TekVISA/include
  )

target_link_libraries(QueryVISA ${VISA_LIBS})
