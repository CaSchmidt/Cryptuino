### Sources ##################################################################

list(APPEND AcquisitionUtil_HEADERS
  include/IInstrument.h
  include/MatOutput.h
  include/Serial.h
  )

list(APPEND AcquisitionUtil_SOURCES
  src/IInstrument.cpp
  src/MatOutput.cpp
  src/Serial.cpp
  )

### Target ###################################################################

add_library(AcquisitionUtil STATIC
  ${AcquisitionUtil_HEADERS}
  ${AcquisitionUtil_SOURCES}
  )

format_output_name(AcquisitionUtil "AcquisitionUtil")

target_include_directories(AcquisitionUtil
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

set_target_properties(AcquisitionUtil PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

target_link_libraries(AcquisitionUtil
  PRIVATE matio
  PUBLIC  CryptuinoUtil
  )
