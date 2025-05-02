ament_add_gtest(simple_maps_tests simple_maps_tests.cpp)
target_include_directories(simple_maps_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(simple_maps_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(simple_maps_tests ${dependencies})
