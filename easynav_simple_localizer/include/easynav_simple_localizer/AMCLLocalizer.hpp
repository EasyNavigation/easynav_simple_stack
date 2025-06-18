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
/// \brief Declaration of the AMCLLocalizer method.

#ifndef EASYNAV_SIMPLE_LOCALIZER__AMCLLOCALIZER_HPP_
#define EASYNAV_SIMPLE_LOCALIZER__AMCLLOCALIZER_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <fstream>
#include <sstream>
#include <random>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "easynav_core/LocalizerMethodBase.hpp"

namespace easynav
{

struct Particle
{
  tf2::Transform pose;
  int hits;
  int possible_hits;
  double weight;
};

/// \brief A localization method implementing a simplified AMCL approach.
class AMCLLocalizer : public LocalizerMethodBase
{
public:
  /**
   * @brief Default constructor.
   */
  AMCLLocalizer();

  /**
   * @brief Destructor.
   */
  ~AMCLLocalizer();

  /**
   * @brief Initializes the maps manager.
   *
   * Creates necessary publishers/subscribers and initializes the map instances.
   *
   * @return std::expected<void, std::string> Success or error string.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  void update_rt(NavState & nav_state) override;
  void update(NavState & nav_state) override;


  tf2::Transform getEstimatedPose() const;
  nav_msgs::msg::Odometry get_pose();

protected:
  void initializeParticles();
  void publishTF(const tf2::Transform & map2bf);
  void publishParticles();
  void publishEstimatedPose(const tf2::Transform & est_pose);

  void predict(NavState & nav_state);
  void correct(NavState & nav_state);
  void reseed();

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimate_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  void odom_callback(nav_msgs::msg::Odometry::UniquePtr msg);

  std::vector<Particle> particles_;
  std::default_random_engine rng_;
  std::shared_ptr<nav_msgs::msg::Odometry> pose_;

  double noise_translation_ {0.01};
  double noise_rotation_ {0.01};
  double noise_translation_to_rotation_ {0.01};
  double min_noise_xy_ {0.05};
  double min_noise_yaw_ {0.05};

  tf2::Transform odom_{tf2::Transform::getIdentity()};
  tf2::Transform last_odom_{tf2::Transform::getIdentity()};
  bool initialized_odom_ = false;

  double reseed_time_;

  rclcpp::Time last_reseed_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__SIMPLEMAPMANAGER_HPP_
