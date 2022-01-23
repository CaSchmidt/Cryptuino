cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

### Project ##################################################################

project(Cryptuino)

# Custom CMake utilities
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(FormatOutputName)

### 3rd Party Dependencies ###################################################

add_subdirectory(${Cryptuino-dev_SOURCE_DIR}/3rdparty/zlib1211
  ${CMAKE_CURRENT_BINARY_DIR}/zlib1211
  )

set(MATIO_MAT73  OFF CACHE BOOL "" FORCE)
set(MATIO_SHARED OFF CACHE BOOL "" FORCE)

add_subdirectory(${Cryptuino-dev_SOURCE_DIR}/3rdparty/matio1521
  ${CMAKE_CURRENT_BINARY_DIR}/mat1521
  )

format_output_name(matio "matio")

add_subdirectory(${Cryptuino-dev_SOURCE_DIR}/csUtil/csUtil
  ${CMAKE_CURRENT_BINARY_DIR}/csUtil
  )

add_subdirectory(${Cryptuino-dev_SOURCE_DIR}/3rdparty/tinyaes10
  ${CMAKE_CURRENT_BINARY_DIR}/tinyaes10
  )

### Projects #################################################################

add_subdirectory(Libraries/CryptuinoUtil)
add_subdirectory(Libraries/AcquisitionUtil)
add_subdirectory(Acquisition/TekVISA)
add_subdirectory(Analysis)
