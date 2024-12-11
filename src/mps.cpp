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

#include "mps_map_gen/mps.hpp"

#include "tf2/utils.h"
namespace mps_map_gen {

MPS::MPS(geometry_msgs::msg::TransformStamped tf, std::string name)
    : MPS(Eigen::Vector2f(tf.transform.translation.x,
                          tf.transform.translation.y),
          Eigen::Rotation2Df(tf2::getYaw(tf.transform.rotation)), name) {}
MPS::MPS(Eigen::Vector2f center, Eigen::Rotation2Df rot, std::string name,
         double width, double length) {
  mps_width = width;
  mps_length = length;
  corners[0] = Eigen::Vector2f(mps_width / 2, -mps_length / 2);
  corners[1] = Eigen::Vector2f(-mps_width / 2, -mps_length / 2);
  corners[2] = Eigen::Vector2f(-mps_width / 2, mps_length / 2);
  corners[3] = Eigen::Vector2f(mps_width / 2, mps_length / 2);

  corners[0] = (rot * corners[0]) + center;
  corners[1] = (rot * corners[1]) + center;
  corners[2] = (rot * corners[2]) + center;
  corners[3] = (rot * corners[3]) + center;

  name_ = name;
  angle = rot.angle();
  center_ = center;
}

//@brief: returns if the mps needs to be set on the map
void MpsMapGenData::set_mps(MPS mps) {
  std::lock_guard<std::mutex> lock(data_mutex);
  std::vector<MPS>::iterator is_in_map =
      std::find(mps_list.begin(), mps_list.end(), mps.name_);
  if (is_in_map == mps_list.end()) {
    mps_list.push_back(mps);
    needs_refresh = true;
    return;
  }

  if (*is_in_map == mps)
    return;

  *is_in_map = mps;
  needs_refresh = true;
}

MPS MPS::from_origin(Eigen::Vector2f origin) const {
  return MPS(
      Eigen::Vector2f(center_.x() - origin.x(), center_.y() - origin.y()),
      Eigen::Rotation2Df(angle), name_, mps_width, mps_length);
}

Eigen::Vector2i MPS::GetMax(float resolution) {
  int maxX = std::max({static_cast<int>(corners[0].x() / resolution),
                       static_cast<int>(corners[1].x() / resolution),
                       static_cast<int>(corners[2].x() / resolution),
                       static_cast<int>(corners[3].x() / resolution)});
  int maxY = std::max({static_cast<int>(corners[0].y() / resolution),
                       static_cast<int>(corners[1].y() / resolution),
                       static_cast<int>(corners[2].y() / resolution),
                       static_cast<int>(corners[3].y() / resolution)});
  return Eigen::Vector2i(maxX, maxY);
}

Eigen::Vector2i MPS::GetMin(float resolution) {
  int minX = std::min({static_cast<int>(corners[0].x() / resolution),
                       static_cast<int>(corners[1].x() / resolution),
                       static_cast<int>(corners[2].x() / resolution),
                       static_cast<int>(corners[3].x() / resolution)});
  int minY = std::min({static_cast<int>(corners[0].y() / resolution),
                       static_cast<int>(corners[1].y() / resolution),
                       static_cast<int>(corners[2].y() / resolution),
                       static_cast<int>(corners[3].y() / resolution)});
  return Eigen::Vector2i(minX, minY);
}
} // namespace mps_map_gen
