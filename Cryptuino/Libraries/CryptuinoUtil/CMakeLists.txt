### Sources ##################################################################

list(APPEND CryptuinoUtil_HEADERS
  include/buffer.h
  include/CmdBooleanOption.h
  include/CmdIntegralOption.h
  include/CmdOption.h
  include/CmdOptions.h
  include/CmdStringOption.h
  include/Randomizer.h
  include/ScopeGuard.h
  )

list(APPEND CryptuinoUtil_SOURCES
  src/CmdBooleanOption.cpp
  src/CmdOption.cpp
  src/CmdOptions.cpp
  src/CmdStringOption.cpp
  src/Randomizer.cpp
  src/ScopeGuard.cpp
  )

### Target ###################################################################

add_library(CryptuinoUtil STATIC
  ${CryptuinoUtil_HEADERS}
  ${CryptuinoUtil_SOURCES}
  )

format_output_name(CryptuinoUtil "CryptuinoUtil")

target_include_directories(CryptuinoUtil
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

set_target_properties(CryptuinoUtil PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

target_link_libraries(CryptuinoUtil
  PUBLIC csUtil
  )
