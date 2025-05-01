// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Declaration of the SimpleMapsManager method.

#ifndef EASYNAV_PLANNER__SIMPLEMAPMANAGER_HPP_
#define EASYNAV_PLANNER__SIMPLEMAPMANAGER_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>  // std::pair
#include <fstream>
#include <sstream>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_common/types/MapTypeBase.hpp"
#include "easynav_simple_common/SimpleMap.hpp"

namespace easynav
{

/**
 * @class SimpleMapsManager
 * @brief A default "simple" implementation for the Planner Method.
 *
 * This planning method does nothing. It serves as an example, and will be used as a default plugin implementation
 * if the navigation system configuration does not specify one.
 */
class SimpleMapsManager : public easynav::MapsManagerBase
{
public:
  SimpleMapsManager() = default;
  ~SimpleMapsManager() = default;

  /**
   * @brief Initialize the planning method.
   *
   * It is not required to override this method. Only if the derived class
   * requires further initialization than the provided by the base class.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Get the current path.
   *
   * This method should return the last path computed.
   * It should not run the planning algorithm (see update method).
   *
   * @return A TwistStamped message with the current path.
   */
  [[nodiscard]] virtual std::shared_ptr<MapsTypeBase> get_static_map() override;

  /**
   * @brief Get the current path.
   *
   * This method should return the last path computed.
   * It should not run the planning algorithm (see update method).
   *
   * @return A TwistStamped message with the current path.
   */
  [[nodiscard]] virtual std::shared_ptr<MapsTypeBase> get_dynamyc_map() override;

  /**
   * @brief Run the path planning method and update the path.
   *
   * This method will be called by the system's PlannerNode to run the planning algorithm.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState & nav_state) override;


  void set_static_map(std::shared_ptr<MapsTypeBase> new_map);
  void set_dynamic_map(std::shared_ptr<MapsTypeBase> new_map);

private:
  /**
   * @brief Current static map.
   */
  std::shared_ptr<SimpleMap> static_map_;

  /**
   * @brief Current static map.
   */
  std::shared_ptr<SimpleMap> dynamic_map_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_occ_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_occ_pub_;
  nav_msgs::msg::OccupancyGrid static_grid_msg_, dynamic_grid_msg_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__SIMPLEMAPMANAGER_HPP_
