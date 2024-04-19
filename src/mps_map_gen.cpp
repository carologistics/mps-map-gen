#include "mps_map_gen/mps_map_gen.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;
namespace mps_map_gen {
MpsMapGen::MpsMapGen() : Node("mps_map_gen") {
  declare_parameter<std::string>("namespace", "");
  declare_parameter<std::string>("map_client", "/map_server/map");
  declare_parameter<bool>("get_data_from_refbox", true);
  declare_parameter<bool>("publish_wait_pos", true);
  declare_parameter<std::string>("peer_address", "127.0.0.1");
  declare_parameter<unsigned short>("recv_port_private", 4441);
  declare_parameter<unsigned short>("recv_port_public", 4444);
  declare_parameter<unsigned short>("field_width", 7);
  declare_parameter<unsigned short>("field_height", 8);
  declare_parameter<std::string>("crypto_key", "randomkey");
  declare_parameter<std::string>(
      "proto_path",
      ament_index_cpp::get_package_share_directory("rcll_protobuf_msgs") +
          "/rcll-protobuf-msgs/");
  namespace_ = this->get_parameter("namespace").as_string();
  data_ = std::make_shared<MpsMapGenData>();
  data_->field_width = get_parameter("field_width").as_int();
  data_->field_height = get_parameter("field_height").as_int();
  use_refbox_data_ = this->get_parameter("get_data_from_refbox").as_bool();
  publish_wait_pos_ = this->get_parameter("publish_wait_pos").as_bool();
  if (publish_wait_pos_) {
    wait_pos_gen_ = std::make_unique<WaitPosGen>(data_);
  }

  std::string peer_address = this->get_parameter("peer_address").as_string();
  unsigned short recv_port_private =
      this->get_parameter("recv_port_private").as_int();
  unsigned short recv_port_public =
      this->get_parameter("recv_port_public").as_int();

  if (use_refbox_data_) {
    RCLCPP_INFO(
        this->get_logger(),
        "Listening to %s on %i (public) and %i (private), using proto Path %s",
        peer_address.c_str(), recv_port_public, recv_port_private,
        this->get_parameter("proto_path").as_string().c_str());
    std::string crypto_key = this->get_parameter("crypto_key").as_string();
    std::string proto_path = this->get_parameter("proto_path").as_string();
    refbox_conn_ = std::make_unique<RefboxConnector>(
        peer_address, recv_port_public, recv_port_private, crypto_key,
        proto_path, data_);
  } else {
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }
  timer_ = this->create_wall_timer(
      500ms, std::bind(&MpsMapGen::update_callback, this));

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  std::string ns = this->get_parameter("namespace").as_string();
  std::string client = this->get_parameter("map_client").as_string();
  map_client =
      this->create_client<nav_msgs::srv::GetMap>(ns + "/map_server/map");

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
      "/mps_map_updates", qos_update);
  map_pubsliher =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mps_map", qos);
  bounded_map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      namespace_+ "/keepout_filter_mask", qos);
  bounded_map_update_publisher =
      this->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
      namespace_+"/keepout_filter_mask_updates", qos_update);

  update_map();

  RCLCPP_INFO(this->get_logger(), "Start");
}

void MpsMapGen::update_callback() {
  if (!use_refbox_data_) {
    std::vector<std::string> frames = tf_buffer->getAllFrameNames();

    for (const auto &frame : frames) {
      if (std::find(std::begin(mps_names), std::end(mps_names), frame) ==
          std::end(mps_names)) {
        continue;
      }
      try {
        mps_transform =
            tf_buffer->lookupTransform("map", frame, tf2::TimePointZero);
        data_->set_mps(MPS(mps_transform, frame));
      } catch (const tf2::TransformException &ex) {
        continue;
      }
    }
  }

  if (data_->needs_refresh) {
    RCLCPP_INFO(this->get_logger(), "Refresh Machine Info");
    update_map();
    publish_tf();
    data_->needs_refresh = false;
  }
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

  // Call the service
  map_client->async_send_request(
      request, std::bind(&MpsMapGen::map_receive, this, std::placeholders::_1));
}

void MpsMapGen::map_receive(
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
  try {
    auto response = future.get();
    RCLCPP_DEBUG(this->get_logger(), "map size %li", response->map.data.size());

    if (response->map.data.size() == 0) {
      data_->needs_refresh = true;
      return;
    }

    double resolution = response->map.info.resolution;
    int map_height = response->map.info.height;
    int map_width = response->map.info.width;

    RCLCPP_INFO(this->get_logger(), "map_height value: %d", map_height);
    RCLCPP_INFO(this->get_logger(), "map_height value: %d", map_width);

    Eigen::Vector2f origin(response->map.info.origin.position.x,
                           response->map.info.origin.position.y);

    for (const auto &mps : data_->mps_list) {
      RCLCPP_INFO(get_logger(), "MPS found %s", mps.name_.c_str());
      add_mps_to_map(mps.from_origin(origin), map_height, map_width, resolution,
                     response->map.data);
    }
    publish_tf();

    // Publish the updated map
    map_update_publisher->publish(
        convertOccupancyGridToOccupancyGridUpdate(response->map));
    map_pubsliher->publish(response->map);

    RCLCPP_INFO(this->get_logger(), std::to_string(response->map.data.size()).c_str());
    auto map_msg = response->map;
    std::vector<int8_t> empty_map(response->map.data.size() , 0);

    RCLCPP_INFO(this->get_logger(), std::to_string(empty_map.size()).c_str());

    add_boundary_to_map(map_height, map_width, resolution, origin, empty_map);
    map_msg.data = empty_map;
    bounded_map_update_publisher->publish(
        convertOccupancyGridToOccupancyGridUpdate(map_msg));
    bounded_map_publisher->publish(map_msg);
    RCLCPP_INFO(this->get_logger(), "published");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service: %s", e.what());
  }
  publish_tf();
}

void MpsMapGen::add_boundary_to_map(int map_height, int map_width,
                                    double resolution,
                                    const Eigen::Vector2f &origin,
                                    std::vector<int8_t> &data) {
  Eigen::Vector2f center(data_->field_width / 2., data_->field_height / 2.);
  center[0] = 0;
  int x_factor = 2;
  MPS new_mps(center, Eigen::Rotation2Df(0.), "map_boundary",
              data_->field_width * x_factor, data_->field_height);
  RCLCPP_INFO(this->get_logger(), "field_height value: %d", data_->field_height);

  RCLCPP_INFO(this->get_logger(), "field_width value: %d", data_->field_width);
  if (!data_->field_mirrored) {
    Eigen::Vector2f center2(data_->field_width / 2., data_->field_height / 2.);
    MPS new_mps2(center2, Eigen::Rotation2Df(0.), "map_boundary",
                 data_->field_width, data_->field_height);
    new_mps2 = new_mps2.from_origin(origin);
    add_mps_to_map(new_mps2, map_height, map_width, resolution, data, 25, false);
  }

  new_mps = new_mps.from_origin(origin);
  add_mps_to_map(new_mps, map_height, map_width, resolution, data, 25, true);
  RCLCPP_INFO(this->get_logger(), "Adding boundary to map");
}

void MpsMapGen::add_mps_to_map(MPS mps, int height, int width,
                               double resolution, std::vector<int8_t> &data,
                               int borderSize, bool inner) {
  float cosAngle = std::cos(mps.angle);
  float sinAngle = std::sin(mps.angle);
  int offset = 25;

  //int borderSize = 3;
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

      // Calculate distances to each side of the rectangle
      float distX = std::abs(cx * cosAngle + cy * sinAngle);
      float distY = std::abs(-cx * sinAngle + cy * cosAngle);
      // Check if the pixel is on the border of the rectangle
      if (std::abs(distX - mps.mps_width / resolution / 2) <= (borderSize) ||
          std::abs(distY - mps.mps_length / resolution / 2) <= (borderSize)) {
        data[py * width + px] = 100; // Set pixel to some value for border
      }
    }
  }
}

void MpsMapGen::publish_tf() {
  // Publish transforms for each machine position
  if (use_refbox_data_) {
    for (const auto &mps : data_->mps_list) {

      // Calculate the rotation angle in radians
      double rotation_radians = mps.angle;

      // Calculate the translation for the input point
      tf2::Vector3 translation_input(
          std::cos(rotation_radians) * (mps.mps_width / 2.0 + approach_dist_),
          std::sin(rotation_radians) * (mps.mps_width / 2.0 + approach_dist_),
          0.0);
      translation_input += tf2::Vector3(mps.center_[0], mps.center_[1], 0.0);
      // Calculate the direction vector from the input point to the center
      tf2::Vector3 direction_input(mps.center_[0], mps.center_[1], 0.0);

      // Create a transform message for machine input
      geometry_msgs::msg::TransformStamped transform_input_msg;
      transform_input_msg.header.stamp = this->now(); // Use current ROS time
      transform_input_msg.header.frame_id = "map";    // Parent frame ID
      transform_input_msg.child_frame_id =
          mps.name_ + "-INPUT"; // Child frame ID
      transform_input_msg.transform.translation.x = translation_input.x();
      transform_input_msg.transform.translation.y = translation_input.y();
      transform_input_msg.transform.translation.z = translation_input.z();

      // Set the orientation to point towards the center
      tf2::Quaternion quaternion_input;
      quaternion_input.setRPY(
          0.0, 0.0,
          std::atan2(direction_input.y() - translation_input.y(),
                     direction_input.x() - translation_input.x()));
      transform_input_msg.transform.rotation = tf2::toMsg(quaternion_input);

      // Publish the transform for the input point
      tf_broadcaster_->sendTransform(transform_input_msg);

      // Calculate the translation for the output point
      tf2::Vector3 translation_output(
          -std::cos(rotation_radians) * (mps.mps_width / 2.0 + approach_dist_),
          -std::sin(rotation_radians) * (mps.mps_width / 2.0 + approach_dist_),
          0.0);
      translation_output += tf2::Vector3(mps.center_[0], mps.center_[1], 0.0);

      // Calculate the direction vector from the output point to the center
      tf2::Vector3 direction_output(mps.center_[0], mps.center_[1], 0.0);

      // Create a transform message for machine output
      geometry_msgs::msg::TransformStamped transform_output_msg;
      transform_output_msg.header.stamp = this->now(); // Use current ROS time
      transform_output_msg.header.frame_id = "map";    // Parent frame ID
      transform_output_msg.child_frame_id =
          mps.name_ + "-OUTPUT"; // Child frame ID
      transform_output_msg.transform.translation.x = translation_output.x();
      transform_output_msg.transform.translation.y = translation_output.y();
      transform_output_msg.transform.translation.z = translation_output.z();

      // Set the orientation to point towards the center
      tf2::Quaternion quaternion_output;
      quaternion_output.setRPY(
          0.0, 0.0,
          std::atan2(direction_output.y() - translation_output.y(),
                     direction_output.x() - translation_output.x()));
      transform_output_msg.transform.rotation = tf2::toMsg(quaternion_output);

      // Publish the transform for the output point
      tf_broadcaster_->sendTransform(transform_output_msg);
      // Also send center
      tf2::Vector3 translation_center =
          tf2::Vector3(mps.center_[0], mps.center_[1], 0.0);
      transform_output_msg.child_frame_id = mps.name_;
      transform_output_msg.transform.translation.x = translation_center.x();
      transform_output_msg.transform.translation.y = translation_center.y();
      transform_output_msg.transform.translation.z = translation_center.z();
      tf_broadcaster_->sendTransform(transform_output_msg);
    }
  }
  if (wait_pos_gen_) {
    std::map<std::string, Eigen::Vector3f> wait_pos =
        wait_pos_gen_->generate_wait_pos(4);
    for (const auto &wait_pos_entry : wait_pos) {
      geometry_msgs::msg::TransformStamped transform_wait_msg;
      transform_wait_msg.header.stamp = this->now(); // Use current ROS time
      transform_wait_msg.header.frame_id = "map";    // Parent frame ID
      transform_wait_msg.child_frame_id = wait_pos_entry.first;
      transform_wait_msg.transform.translation.x = wait_pos_entry.second[0];
      transform_wait_msg.transform.translation.y = wait_pos_entry.second[1];
      transform_wait_msg.transform.translation.z = 0.0;
      tf2::Quaternion quat;
      quat.setRPY(
          0, 0,
          wait_pos_entry.second[2]); // Set roll, pitch, and yaw (only yaw is
                                     // non-zero for rotation around z-axis)
      transform_wait_msg.transform.rotation = tf2::toMsg(quat);
      tf_broadcaster_->sendTransform(transform_wait_msg);
    }
  }
}

} // namespace mps_map_gen

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mps_map_gen::MpsMapGen>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
