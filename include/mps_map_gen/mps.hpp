// Copyright (c) 2024 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mps_map_gen {

class MPS {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MPS(geometry_msgs::msg::TransformStamped tf, std::string name);
  MPS(Eigen::Vector2f center, Eigen::Rotation2Df rot, std::string name,
      double width = MPS::default_mps_width,
      double length = MPS::default_mps_length);

  static constexpr double default_mps_length = 0.7;
  static constexpr double default_mps_width = 0.35;

  MPS from_origin(Eigen::Vector2f origin) const;

  Eigen::Vector2f corners[4];
  Eigen::Vector2f center_;
  double mps_width;
  double mps_length;
  float angle;
  std::string name_;

  bool operator==(const MPS mps) {
    return this->center_.x() == mps.center_.x() && this->angle == mps.angle &&
           this->center_.y() == mps.center_.y();
  }

  bool operator==(const std::string name) { return this->name_ == name; }
  Eigen::Vector2i GetMax(float resolution);
  Eigen::Vector2i GetMin(float resolution);
};
struct MpsMapGenData {
  bool needs_refresh = true;
  bool team_magenta = false;
  int field_width = 0;
  int field_height = 0;
  bool field_mirrored = true;
  std::vector<MPS> mps_list;
  std::mutex data_mutex;
  void set_mps(MPS mps);
};
} // namespace mps_map_gen
