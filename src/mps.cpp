#include "mps_map_gen/mps.hpp"
#include "mps_map_gen/mps_map_gen.hpp"

#include "tf2/utils.h"
namespace mps_map_gen {

MPS::MPS(geometry_msgs::msg::TransformStamped tf, std::string name)
    : MPS(Eigen::Vector2f(tf.transform.translation.x,
                          tf.transform.translation.y),
          Eigen::Rotation2Df(tf2::getYaw(tf.transform.rotation)), name) {}
MPS::MPS(Eigen::Vector2f center, Eigen::Rotation2Df rot, std::string name) {
  name_ = name;
  center = Eigen::Vector2f(center.x() + 9.0f, center.y() + 1.5f);
  double mps_width = MpsMapGen::mps_width;
  double mps_length = MpsMapGen::mps_length;
  corners[0] = Eigen::Vector2f(mps_width / 2, -mps_length / 2);
  corners[1] = Eigen::Vector2f(-mps_width / 2, -mps_length / 2);
  corners[2] = Eigen::Vector2f(-mps_width / 2, mps_length / 2);
  corners[3] = Eigen::Vector2f(mps_width / 2, mps_length / 2);

  corners[0] = (rot * corners[0]) + center;
  corners[1] = (rot * corners[1]) + center;
  corners[2] = (rot * corners[2]) + center;
  corners[3] = (rot * corners[3]) + center;
  angle = rot.angle();
  center_ = center;
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