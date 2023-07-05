#include "mps_map_gen/mps_map_gen.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
namespace mps_map_gen {
MpsMapGen::MpsMapGen() : Node("mps_map_gen") {
  tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  map_client = this->create_client<nav_msgs::srv::GetMap>("map_server/map");

  // Set QoS to match the expected settings for map visualization in RViz
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.history(rclcpp::HistoryPolicy::KeepLast);

  rclcpp::QoS qos_update(
      rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_update.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_update.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos_update.history(rclcpp::HistoryPolicy::KeepLast);
  qos_update.keep_last(24);

  map_update_publisher =
      this->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
          "mps_map_updates", qos_update);
  map_pubsliher =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("mps_map", qos);

  update_map();
  timer_ =
      this->create_wall_timer(500ms, std::bind(&MpsMapGen::tfCallback, this));

  RCLCPP_INFO(this->get_logger(), "Start");
}

void MpsMapGen::tfCallback() {
  std::vector<std::string> frames = tf_buffer->getAllFrameNames();
  ;
  bool needs_refresh = false;
  for (const auto &frame : frames) {
    if (frame == "map" || frame == "mps")
      continue;
    if (tf_buffer->canTransform("mps", frame, tf2::TimePointZero) &&
        tf_buffer->canTransform("map", frame, tf2::TimePointZero)) {
      try {
        mps_transform =
            tf_buffer->lookupTransform("map", frame, tf2::TimePointZero);
        bool new_mps = set_mps(MPS(mps_transform, frame));
        if (new_mps)
          needs_refresh = true;
      } catch (const tf2::TransformException &ex) {
        continue;
      }
    }
  }

  if (needs_refresh)
    update_map();
}

map_msgs::msg::OccupancyGridUpdate convertOccupancyGridToOccupancyGridUpdate(
    const nav_msgs::msg::OccupancyGrid &occupancy_grid) {
  map_msgs::msg::OccupancyGridUpdate occupancy_grid_update;

  // The header for the update is the same as the header for the full map
  occupancy_grid_update.header = occupancy_grid.header;

  // Set the region of the update. Here, we're simply updating the entire map.
  // In a real use case, you would likely only update a portion of the map.
  occupancy_grid_update.x = 0;
  occupancy_grid_update.y = 0;
  occupancy_grid_update.width = occupancy_grid.info.width;
  occupancy_grid_update.height = occupancy_grid.info.height;

  // The update data is the same as the map data
  occupancy_grid_update.data = occupancy_grid.data;

  return occupancy_grid_update;
}

MpsMapGen::~MpsMapGen() {}

void MpsMapGen::update_map() {
  RCLCPP_INFO(this->get_logger(), "Update");
  // Wait for the service to become available
  while (!map_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
  }

  // Create a request object
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  RCLCPP_INFO(this->get_logger(), "Request");

  // Call the service
  map_client->async_send_request(
      request, std::bind(&MpsMapGen::map_receive, this, std::placeholders::_1));
}
void MpsMapGen::map_receive(
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
  try {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "map");

    double resolution = response->map.info.resolution;
    int map_height = response->map.info.height;
    int map_width = response->map.info.width;

    for (const auto &mps : mps_list) {
      add_mps_to_map(mps, map_height, map_width, resolution,
                     response->map.data);
    }

    // Publish the updated map
    map_update_publisher->publish(
        convertOccupancyGridToOccupancyGridUpdate(response->map));
    map_pubsliher->publish(response->map);
    RCLCPP_INFO(this->get_logger(), "published");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service: %s", e.what());
  }
}

void MpsMapGen::add_mps_to_map(MPS mps, int height, int width,
                               double resolution, std::vector<int8_t> &data) {
  // Calculate the bounding box of the rotated box

  float cosAngle = std::cos(mps.angle);
  float sinAngle = std::sin(mps.angle);
  // Clamp the bounding box to image boundaries
  int minX = std::max(0, mps.GetMin(resolution).x());
  int minY = std::max(0, mps.GetMin(resolution).y());
  int maxX = std::min(width - 1, mps.GetMax(resolution).x());
  int maxY = std::min(height - 1, mps.GetMax(resolution).y());

  // Iterate over the pixels within the bounding box
  for (int py = minY; py <= maxY; ++py) {
    for (int px = minX; px <= maxX; ++px) {
      int mps_x = static_cast<int>(mps.center_.x() / resolution);
      int mps_y = static_cast<int>(mps.center_.y() / resolution);
      int cx = px - mps_x;
      int cy = py - mps_y;

      // Check if the pixel is inside the rotated box
      if (std::abs(cx * cosAngle + cy * sinAngle) <=
              mps_width / resolution / 2 &&
          std::abs(-cx * sinAngle + cy * cosAngle) <=
              mps_length / resolution / 2) {
        data[py * width + px] = 100;
      }
    }
  }
}
//@brief: returns if the mps needs to be set on the map
bool MpsMapGen::set_mps(MPS mps) {
  std::vector<MPS>::iterator is_in_map =
      std::find(mps_list.begin(), mps_list.end(), mps.name_);
  if (is_in_map == mps_list.end()) {
    mps_list.push_back(mps);
    return true;
  }

  if (*is_in_map == mps)
    return false;

  *is_in_map = mps;
  return true;
}

} // namespace mps_map_gen

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mps_map_gen::MpsMapGen>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
