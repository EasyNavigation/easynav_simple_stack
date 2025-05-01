// Copyright 2025 Intelligent Robotics Lab
//
// Licensed under the GNU General Public License v3.0.

#include <gtest/gtest.h>

#include "easynav_simple_common/SimpleMap.hpp"
#include "easynav_simple_maps_manager/SimpleMapsManager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>

/// \brief Fixture for SimpleMapsManager tests (minimal)
class SimpleMapsManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};


/// \brief Dynamic map update tests
TEST_F(SimpleMapsManagerTest, BasicDynamicUpdate)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  auto manager = std::make_shared<easynav::SimpleMapsManager>();
  manager->initialize(node, "test");

  auto static_map = std::make_shared<easynav::SimpleMap>();
  static_map->initialize(30, 30, 0.1, -1.5, -1.5, 0.0);
  manager->set_static_map(static_map);

  easynav::NavState navstate;
  auto perception = std::make_shared<easynav::Perception>();

  perception->data.points.resize(6);
  perception->data.points[0].x = 1.0;
  perception->data.points[0].y = 1.0;
  perception->data.points[0].z = 0.0;
  perception->data.points[1].x = -1.0;
  perception->data.points[1].y = -1.0;
  perception->data.points[1].z = 0.0;
  perception->data.points[2].x = -10.0;
  perception->data.points[2].y = -1.0;
  perception->data.points[2].z = 0.0;
  perception->data.points[3].x = 10.0;
  perception->data.points[3].y = -1.0;
  perception->data.points[3].z = 0.0;
  perception->data.points[4].x = 1.0;
  perception->data.points[4].y = -10.0;
  perception->data.points[4].z = 0.0;
  perception->data.points[5].x = 1.0;
  perception->data.points[5].y = 10.0;
  perception->data.points[5].z = 0.0;

  perception->stamp = rclcpp::Time(0);
  perception->frame_id = "map";
  perception->valid = true;
  navstate.perceptions.push_back(perception);

  manager->update(navstate);

  auto map_ptr = std::dynamic_pointer_cast<easynav::SimpleMap>(manager->get_dynamyc_map());
  ASSERT_TRUE(map_ptr != nullptr);

  auto cell1 = map_ptr->metric_to_cell(1.0, 1.0);
  EXPECT_TRUE(map_ptr->at(cell1.first, cell1.second));

  auto cell2 = map_ptr->metric_to_cell(-1.0, -1.0);
  EXPECT_TRUE(map_ptr->at(cell2.first, cell2.second));
}
