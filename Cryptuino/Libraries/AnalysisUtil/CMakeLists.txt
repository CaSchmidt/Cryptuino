### Sources ##################################################################

list(APPEND AnalysisUtil_HEADERS
  include/Campaign.h
  include/CampaignReader.h
  include/CPA.h
  include/IPowerModel.h
  include/ITrigger.h
  include/MatInput.h
  include/Matrix.h
  include/PowerAES.h
  include/TriggerImpl.h
  include/TriggerSelect.h
  )

list(APPEND AnalysisUtil_SOURCES
  src/Campaign.cpp
  src/CampaignReader.cpp
  src/CPA.cpp
  src/IPowerModel.cpp
  src/ITrigger.cpp
  src/MatInput.cpp
  src/PowerAES.cpp
  src/TriggerImpl.cpp
  src/TriggerSelect.cpp
  )

### Target ###################################################################

add_library(AnalysisUtil STATIC
  ${AnalysisUtil_HEADERS}
  ${AnalysisUtil_SOURCES}
  )

format_output_name(AnalysisUtil "AnalysisUtil")

target_include_directories(AnalysisUtil
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

set_target_properties(AnalysisUtil PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

target_link_libraries(AnalysisUtil
  PRIVATE matio
  PUBLIC  CryptuinoUtil
  )
