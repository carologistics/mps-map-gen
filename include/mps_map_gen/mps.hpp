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
  MPS(Eigen::Vector2f center, Eigen::Rotation2Df rot, std::string name);

  MPS from_origin(Eigen::Vector2f origin) const;

  Eigen::Vector2f corners[4];
  Eigen::Vector2f center_;
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
} // namespace mps_map_gen
