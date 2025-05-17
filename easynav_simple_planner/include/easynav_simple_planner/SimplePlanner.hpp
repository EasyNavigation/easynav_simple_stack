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
/// \brief Declaration of the SimplePlanner class implementing A* path planning.

#ifndef EASYNAV_PLANNER__SIMPLEPLANNER_HPP_
#define EASYNAV_PLANNER__SIMPLEPLANNER_HPP_

#include <memory>
#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_simple_common/SimpleMap.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/// \brief Planner based on A* algorithm using SimpleMap
class SimplePlanner : public PlannerMethodBase
{
public:
  /// \brief Default constructor
  explicit SimplePlanner() = default;

  /**
   * @brief Initializes the planner.
   *
   * Creates necessary publishers/subscribers and initializes the map instances.
   *
   * @return std::expected<void, std::string> Success or error string.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /// \brief Computes a path using A* algorithm
  /// \param nav_state Current navigation state (with odometry and goals)
  void update(const NavState & nav_state) override;

  /// \brief Returns the last computed path
  /// \return nav_msgs::msg::Path with the poses from the last planning cycle
  nav_msgs::msg::Path get_path() override;

protected:
  double robot_radius_;
  nav_msgs::msg::Path current_path_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /// \brief Internal A* implementation
  /// \param map The binary occupancy map
  /// \param start Pose in world coordinates
  /// \param goal Pose in world coordinates
  /// \param resolution Cell size in meters
  /// \return List of poses representing the path
  std::vector<geometry_msgs::msg::Pose> a_star_path(
    const SimpleMap & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    double resolution);

  /// \brief Checks if a cell is free considering a clearance radius
  /// \param map The map to query
  /// \param cx Cell x-coordinate
  /// \param cy Cell y-coordinate
  /// \param clearance_cells Minimum clearance radius in cells
  /// \return true if the area is free, false if any nearby cell is occupied
  bool isFreeWithClearance(
    const SimpleMap & map,
    int cx, int cy,
    double clearance_cells);
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__SIMPLEPLANNER_HPP_
