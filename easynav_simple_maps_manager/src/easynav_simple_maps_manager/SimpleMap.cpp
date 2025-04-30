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


#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>  // std::pair
#include <fstream>
#include <sstream>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "easynav_common/types/MapTypeBase.hpp"
#include "easynav_simple_maps_manager/SimpleMap.hpp"

namespace easynav
{

SimpleMap::SimpleMap()
: width_(0), height_(0), resolution_(1.0), origin_x_(0.0), origin_y_(0.0), data_()
{}

void
SimpleMap::initialize(
  std::size_t width, std::size_t height, double resolution,
  double origin_x, double origin_y, bool initial_value)
{
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  data_.assign(width * height, initial_value);
}

uint8_t
SimpleMap::at(std::size_t x, std::size_t y) const
{
  check_bounds(x, y);
  return data_[index(x, y)];
}


uint8_t &
SimpleMap::at(std::size_t x, std::size_t y)
{
  check_bounds(x, y);
  return data_.at(index(x, y));
}

void
SimpleMap::fill(uint8_t value)
{
  std::fill(data_.begin(), data_.end(), value);
}

void
SimpleMap::deep_copy(const SimpleMap & other)
{
  width_ = other.width_;
  height_ = other.height_;
  resolution_ = other.resolution_;
  origin_x_ = other.origin_x_;
  origin_y_ = other.origin_y_;
  data_ = other.data_;
}

bool
SimpleMap::check_bounds_metric(double mx, double my) const
{
  double relative_x = mx - origin_x_;
  double relative_y = my - origin_y_;

  if (relative_x < 0.0 || relative_y < 0.0) {
    return false;
  }

  std::size_t x = static_cast<std::size_t>(relative_x / resolution_);
  std::size_t y = static_cast<std::size_t>(relative_y / resolution_);

  return x < width_ && y < height_;
}

std::pair<double, double>
SimpleMap::cell_to_metric(std::size_t x, std::size_t y) const
{
  double mx = origin_x_ + (static_cast<double>(x) + 0.5) * resolution_;
  double my = origin_y_ + (static_cast<double>(y) + 0.5) * resolution_;
  return {mx, my};
}

std::pair<std::size_t, std::size_t>
SimpleMap::metric_to_cell(double mx, double my) const
{
  double relative_x = mx - origin_x_;
  double relative_y = my - origin_y_;

  std::size_t x = static_cast<std::size_t>(relative_x / resolution_);
  std::size_t y = static_cast<std::size_t>(relative_y / resolution_);

  return {x, y};
}

void
SimpleMap::to_occupancy_grid(nav_msgs::msg::OccupancyGrid & grid_msg) const
{
  grid_msg.info.width = static_cast<uint32_t>(width_);
  grid_msg.info.height = static_cast<uint32_t>(height_);
  grid_msg.info.resolution = static_cast<float>(resolution_);


  grid_msg.info.origin.position.x = origin_x_;
  grid_msg.info.origin.position.y = origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.x = 0.0;
  grid_msg.info.origin.orientation.y = 0.0;
  grid_msg.info.origin.orientation.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  if (grid_msg.data.size() != width_ * height_) {
    grid_msg.data.resize(width_ * height_);
  }

  for (std::size_t idx = 0; idx < data_.size(); ++idx) {
    grid_msg.data[idx] = data_[idx] ? 100 : 0;
  }
}


bool
SimpleMap::save_to_file(const std::string & path) const
{
  std::ofstream out(path);
  if (!out.is_open()) {
    return false;
  }

  out << width_ << " " << height_ << " "
      << resolution_ << " "
      << origin_x_ << " " << origin_y_ << "\n";

  for (std::size_t i = 0; i < data_.size(); ++i) {
    out << (data_[i] ? "1" : "0");
    if (i + 1 < data_.size()) {
      out << " ";
    }
  }

  out << "\n";
  return true;
}


bool
SimpleMap::load_from_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in.is_open()) {
    return false;
  }

  std::string line;

  if (!std::getline(in, line)) {return false;}
  std::istringstream meta_stream(line);
  std::size_t w, h;
  double res, ox, oy;
  if (!(meta_stream >> w >> h >> res >> ox >> oy)) {
    return false;
  }

  if (!std::getline(in, line)) {return false;}
  std::istringstream data_stream(line);

  std::vector<uint8_t> new_data;
  int val;
  while (data_stream >> val) {
    new_data.push_back(val == 1);
  }

  if (new_data.size() != w * h) {
    return false;
  }

  width_ = w;
  height_ = h;
  resolution_ = res;
  origin_x_ = ox;
  origin_y_ = oy;
  data_ = std::move(new_data);

  return true;
}

std::size_t
SimpleMap::index(std::size_t x, std::size_t y) const
{
  return y * width_ + x;  // Row-major order
}

void
SimpleMap::check_bounds(std::size_t x, std::size_t y) const
{
  if (x >= width_ || y >= height_) {
    throw std::out_of_range("SimpleMap: index out of bounds");
  }
}

/**
 * @brief Prints metadata and optionally all map cell values with coordinates.
 *
 * @param view_data If true, prints cell-by-cell content with coordinates. Default is false.
 */
void
SimpleMap::print(bool view_data) const
{
  std::cerr << "SimpleMap Metadata:\n";
  std::cerr << "  Size: " << width_ << " x " << height_ << "\n";
  std::cerr << "  Resolution: " << resolution_ << " m/cell\n";
  std::cerr << "  Origin: (" << origin_x_ << ", " << origin_y_ << ")\n";

  if (!view_data) {return;}

  std::cerr << "SimpleMap Data:\n";
  for (std::size_t y = 0; y < height_; ++y) {
    for (std::size_t x = 0; x < width_; ++x) {
      double mx = origin_x_ + (x + 0.5) * resolution_;
      double my = origin_y_ + (y + 0.5) * resolution_;
      std::cerr << "[" << x << ", " << y << "][" << mx << ", " << my << "] "
                << static_cast<int>(data_[index(x, y)]) << "\n";
    }
  }
}


}  // namespace easynav
