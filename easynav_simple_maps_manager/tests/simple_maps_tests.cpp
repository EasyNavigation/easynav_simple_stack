// Copyright 2025 Intelligent Robotics Lab
//
// Licensed under the GNU General Public License v3.0.

#include <gtest/gtest.h>

#include "easynav_simple_maps_manager/SimpleMap.hpp"
#include "easynav_simple_maps_manager/SimpleMapsManager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <memory>

/// \brief Fixture for SimpleMap tests (minimal)
class SimpleMapTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

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

/// \brief Basic access and fill tests
TEST_F(SimpleMapTest, BasicAccessAndFill)
{
  easynav::SimpleMap map;
  map.initialize(5, 5, 0.5, -1.0, -1.0);

  EXPECT_EQ(map.width(), 5u);
  EXPECT_EQ(map.height(), 5u);
  EXPECT_DOUBLE_EQ(map.resolution(), 0.5);
  EXPECT_DOUBLE_EQ(map.origin_x(), -1.0);
  EXPECT_DOUBLE_EQ(map.origin_y(), -1.0);

  for (std::size_t x = 0; x < 5; ++x) {
    for (std::size_t y = 0; y < 5; ++y) {
      EXPECT_FALSE(map.at(x, y));
    }
  }

  map.at(2, 2) = true;
  EXPECT_TRUE(map.at(2, 2));

  map.fill(true);
  for (std::size_t x = 0; x < 5; ++x) {
    for (std::size_t y = 0; y < 5; ++y) {
      EXPECT_TRUE(map.at(x, y));
    }
  }
}

/// \brief Metric conversion tests
TEST_F(SimpleMapTest, MetricConversion)
{
  easynav::SimpleMap map;
  map.initialize(10, 10, 1.0, -5.0, -5.0);

  auto metric = map.cell_to_metric(0, 0);
  EXPECT_NEAR(metric.first, -4.5, 1e-6);
  EXPECT_NEAR(metric.second, -4.5, 1e-6);

  auto cell = map.metric_to_cell(-4.6, -4.6);
  EXPECT_EQ(cell.first, 0u);
  EXPECT_EQ(cell.second, 0u);

  EXPECT_TRUE(map.check_bounds_metric(-4.6, -4.6));
  EXPECT_FALSE(map.check_bounds_metric(-6.0, -6.0));
}

/// \brief Save and load map tests
TEST_F(SimpleMapTest, SaveLoadMap)
{
  easynav::SimpleMap map;
  map.initialize(4, 4, 0.5, -1.0, -1.0);
  map.at(1, 1) = true;
  map.at(2, 2) = true;

  ASSERT_TRUE(map.save_to_file("test_map.txt"));

  easynav::SimpleMap loaded_map;
  ASSERT_TRUE(loaded_map.load_from_file("test_map.txt"));

  EXPECT_EQ(loaded_map.width(), 4u);
  EXPECT_EQ(loaded_map.height(), 4u);
  EXPECT_DOUBLE_EQ(loaded_map.resolution(), 0.5);
  EXPECT_DOUBLE_EQ(loaded_map.origin_x(), -1.0);
  EXPECT_DOUBLE_EQ(loaded_map.origin_y(), -1.0);

  EXPECT_TRUE(loaded_map.at(1, 1));
  EXPECT_TRUE(loaded_map.at(2, 2));
  EXPECT_FALSE(loaded_map.at(0, 0));
}

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
