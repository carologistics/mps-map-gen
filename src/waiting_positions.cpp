#include "mps_map_gen/waiting_positions.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcll_protobuf_cpp/MachineInfo.pb.h"
#include "rcll_protobuf_cpp/VersionInfo.pb.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <queue>

using namespace std::chrono_literals;
using std::placeholders::_1;
namespace mps_map_gen {
WaitPosGen::WaitPosGen(std::shared_ptr<MpsMapGenData> data) { data_ = data; }

// Function to find the cell with the maximum distance to occupied cells
std::map<std::string, Eigen::Vector3f>
WaitPosGen::generate_free_wait_pos(const std::vector<std::vector<bool>> &grid,
                                   int num_pos) {
  const int numRows = grid.size();
  const int numCols = grid[0].size();
  const int maxDistance = numRows * numCols;
  int x_min = data_->field_mirrored ? (-data_->field_width) : (0);
  std::vector<std::vector<int>> distances(numRows, std::vector<int>(numCols));

  // Initialize the distance grid
  for (int i = 0; i < numRows; ++i) {
    for (int j = 0; j < numCols; ++j) {
      if (grid[i][j]) {
        distances[i][j] = 0; // Distance is 0 for obstacle cells
      } else {
        distances[i][j] = maxDistance; // Set initial distance to maximum
      }
    }
  }

  // Perform value iteration
  bool changed;
  do {
    changed = false;
    for (int i = 0; i < numRows; ++i) {
      for (int j = 0; j < numCols; ++j) {
        if (!grid[i][j]) { // Update only for free cells
          int newDistance = maxDistance;
          // Check neighbors
          if (i > 0) {
            newDistance = std::min(newDistance, distances[i - 1][j] + 1);
          }
          if (i < numRows - 1) {
            newDistance = std::min(newDistance, distances[i + 1][j] + 1);
          }
          if (j > 0) {
            newDistance = std::min(newDistance, distances[i][j - 1] + 1);
          }
          if (j < numCols - 1) {
            newDistance = std::min(newDistance, distances[i][j + 1] + 1);
          }
          // Update if the new distance is smaller
          if (newDistance < distances[i][j]) {
            distances[i][j] = newDistance;
            changed = true;
          }
        }
      }
    }
  } while (changed);

  // Priority queue to store the n best distances along with their indices
  auto cmp = [](const std::tuple<int, int, int> &a,
                const std::tuple<int, int, int> &b) {
    return std::get<2>(a) > std::get<2>(b); // Min-heap based on distance
  };
  std::priority_queue<std::tuple<int, int, int>,
                      std::vector<std::tuple<int, int, int>>, decltype(cmp)>
      pq(cmp);

  // Find the n best distances
  for (int i = 0; i < numRows; ++i) {
    for (int j = 0; j < numCols; ++j) {
      pq.push(std::make_tuple(i, j, distances[i][j]));
      if (pq.size() >
          static_cast<size_t>(
              num_pos)) { // Keep only the top n elements in the priority queue
        pq.pop();
      }
    }
  }

  // Extract the best n results from the priority queue
  std::map<std::string, Eigen::Vector3f> res;
  int count = 1;
  while (!pq.empty()) {
    auto [i, j, dist] = pq.top();
    int offset = 1;
    if (i + x_min < 0) {
      offset = 0;
    }
    auto mid_point = zone_to_point(Eigen::Vector2i(i + x_min + offset, j + 1));
    res.emplace("WAIT" + std::to_string(count),
                Eigen::Vector3f(mid_point[0], mid_point[1], 0));
    pq.pop();
    count++;
  }
  return res;
}

std::vector<Eigen::Vector2i> WaitPosGen::blocked_zones(const MPS &mps) {
  std::vector<Eigen::Vector2i> rv;
  rv.push_back(point_to_zone(mps.center_));
  uint16_t discrete_ori = discrete_angle_from_yaw(mps.angle);
  for (const Eigen::Vector2i &offset : zone_blocking_[discrete_ori % 180]) {
    rv.push_back(point_to_zone(mps.center_) + offset);
  }
  return rv;
}

inline uint16_t WaitPosGen::discrete_angle_from_yaw(float yaw) {
  if (yaw < 0)
    yaw = 2.0f * float(M_PI) + yaw;
  uint16_t yaw_discrete = uint16_t(std::round(yaw / float(M_PI) * 4)) * 45;
  return yaw_discrete == 360 ? 0 : yaw_discrete;
}
Eigen::Vector2i WaitPosGen::point_to_zone(const Eigen::Vector2f &p) {
  return Eigen::Vector2i(p[0] < 0 ? std::floor(p[0]) : std::ceil(p[0]),
                         std::ceil(p[1]));
}
Eigen::Vector2f WaitPosGen::zone_to_point(const Eigen::Vector2i &p) {
  return Eigen::Vector2f(p[0] < 0 ? (p[0] + 0.5) : (p[0] - 0.5), p[1] - 0.5);
}
std::map<std::string, Eigen::Vector3f>
WaitPosGen::generate_wait_pos(int num_free_zones) {
  std::map<std::string, Eigen::Vector3f> res;
  if (data_->mps_list.size() == 0 || data_->field_width == 0 ||
      data_->field_height == 0) {
    return res;
  }
  std::vector<Eigen::Vector2i> blocked_zones;
  for (const auto &mps : data_->mps_list) {
    blocked_zones.push_back(point_to_zone(mps.center_));
    std::vector<Eigen::Vector2i> blocked_mps_zones = this->blocked_zones(mps);
    blocked_zones.insert(blocked_zones.end(), blocked_mps_zones.begin(),
                         blocked_mps_zones.end());
  }

  std::vector<Eigen::Vector2i> free_zones;

  int x_min = 0;
  int x_max = data_->field_width;
  int y_min = 0;
  int y_max = data_->field_height;
  std::vector<Eigen::Vector2i> reserved_zones = {
      {x_max - 2, 2},
      {x_max - 2, 1},
      {x_max - 1, 1},
      {x_max, 1},
  };
  if (data_->field_mirrored) {
    x_min = -x_max;
    std::vector<Eigen::Vector2i> negative_reserved_zones = {
        {-(x_max - 2), 2},
        {-(x_max - 2), 1},
        {-(x_max - 1), 1},
        {-(x_max), 1},
    };
    reserved_zones.insert(reserved_zones.end(), negative_reserved_zones.begin(),
                          negative_reserved_zones.end());
  }
  std::vector<std::vector<bool>> occupancy_grid(
      -x_min + x_max, std::vector<bool>(y_max, false));
  for (int x = x_min; x <= x_max; ++x) {
    for (int y = y_min; y <= y_max; ++y) {
      if (x != 0 && y != 0) {
        Eigen::Vector2i zn = {x, y};
        if (std::find(reserved_zones.begin(), reserved_zones.end(), zn) ==
                reserved_zones.end() &&
            std::find(blocked_zones.begin(), blocked_zones.end(), zn) ==
                blocked_zones.end()) {
          // Zone is not reserved or blocked
          free_zones.push_back(zn);
        } else {
          int tmp = x < 0 ? 0 : 1;
          occupancy_grid[x - x_min - tmp][y - 1] = true;
        }
      }
    }
  }
  auto mps_wait_pos = generate_mps_wait_pos(free_zones);
  auto free_wait_pos = generate_free_wait_pos(occupancy_grid, num_free_zones);
  mps_wait_pos.merge(free_wait_pos);
  return mps_wait_pos;
}

std::map<std::string, Eigen::Vector3f>
WaitPosGen::generate_mps_wait_pos(std::vector<Eigen::Vector2i> &free_zones) {
  std::map<std::string, Eigen::Vector3f> res;

  for (auto &mps : data_->mps_list) {
    Eigen::Vector2f direction(std::cos(mps.angle), std::sin(mps.angle));
    Eigen::Vector2f input_pos = mps.center_ + direction;
    Eigen::Vector2f output_pos = mps.center_ - direction;
    if (mps.name_ == "C-BS") {
    }

    std::sort(
        free_zones.begin(), free_zones.end(),
        [input_pos](const Eigen::Vector2i &lhs, const Eigen::Vector2i &rhs) {
          double d_lhs = (lhs.cast<float>() - input_pos).norm();
          double d_rhs = (rhs.cast<float>() - input_pos).norm();
          return d_lhs < d_rhs;
        });
    Eigen::Vector2f ori_vec = (*(free_zones.begin())).cast<float>() - input_pos;
    double wait_ori = std::atan2(ori_vec.x(), ori_vec.y());
    auto mid_point_input = zone_to_point(*(free_zones.begin()));
    res.emplace(
        mps.name_ + "-I-WAIT",
        Eigen::Vector3f(mid_point_input[0], mid_point_input[1], wait_ori));
    std::sort(
        free_zones.begin(), free_zones.end(),
        [output_pos](const Eigen::Vector2i &lhs, const Eigen::Vector2i &rhs) {
          double d_lhs = (lhs.cast<float>() - output_pos).norm();
          double d_rhs = (rhs.cast<float>() - output_pos).norm();
          return d_lhs < d_rhs;
        });
    ori_vec = (*(free_zones.begin())).cast<float>() - output_pos;
    wait_ori = std::atan2(ori_vec.x(), ori_vec.y());
    auto mid_point_output = zone_to_point(*(free_zones.begin()));
    res.emplace(
        mps.name_ + "-O-WAIT",
        Eigen::Vector3f(mid_point_output[0], mid_point_output[1], wait_ori));
  }
  return res;
}

} // namespace mps_map_gen
