ament_add_gtest(simple_mapsmanager_tests simple_mapsmanager_tests.cpp)
target_include_directories(simple_mapsmanager_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(simple_mapsmanager_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(simple_mapsmanager_tests ${dependencies})
