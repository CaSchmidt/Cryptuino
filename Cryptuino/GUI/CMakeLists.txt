list(APPEND gui_FORMS
  forms/WMainWindow.ui
  )

list(APPEND gui_HEADERS
  include/WMainWindow.h
  )

list(APPEND gui_SOURCES
  src/main.cpp
  src/WMainWindow.cpp
  )

### Target ###################################################################

add_executable(gui
  ${gui_FORMS}
  ${gui_HEADERS}
  ${gui_SOURCES}
  )

format_output_name(gui "AnalysisGUI")

if(WIN32)
  set_target_properties(gui PROPERTIES
    WIN32_EXECUTABLE ON
    )
endif()

target_compile_definitions(gui
  PRIVATE QT_NO_CAST_FROM_ASCII
  PRIVATE QT_NO_CAST_TO_ASCII
  )

target_include_directories(gui
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_link_libraries(gui
  PRIVATE AnalysisUtil
  PRIVATE csUtil
  PRIVATE Qt5::Widgets
  )

set_target_properties(gui PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  )

set_target_properties(gui PROPERTIES
  AUTOMOC ON
  AUTORCC ON
  AUTOUIC ON
  )

set_property(TARGET gui APPEND PROPERTY
  AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/forms
  )
