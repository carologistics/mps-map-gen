// Licensed under GPLv2+. See LICENSE file. Copyright Carologistics.

#ifndef MPS_MAP_GEN_H
#define MPS_MAP_GEN_H

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/Geometry>
#include <boost/asio.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "mps.hpp"
#include "mps_map_gen/refbox_connector.hpp"
#include "mps_map_gen/waiting_positions.hpp"

namespace mps_map_gen {
class MpsMapGen : public rclcpp::Node {
public:
  MpsMapGen();

  ~MpsMapGen();

private:
  void update_map();
  void add_mps_to_map(MPS mps, int height, int width, double resolution,
                      std::vector<int8_t> &data, int thickness = 3);
  void map_receive(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future);
  bool set_mps(MPS mps);
  void update_callback();
  void add_boundary_to_map(int map_height, int map_width, double resolution,
                           const Eigen::Vector2f &origin,
                           std::vector<int8_t> &data);

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr mps_tf;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
      map_update_publisher;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pubsliher;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      bounded_map_publisher;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
      bounded_map_update_publisher;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr static_tf_timer_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string mps_names[14] = {"M-BS",  "M-SS",  "M-DS",  "M-CS1", "M-CS2",
                               "M-RS1", "M-RS2", "C-BS",  "C-SS",  "C-DS",
                               "C-CS1", "C-CS2", "C-RS1", "C-RS2"};
  std::string namespace_;
  void publish_tf();

  // only here to prevent realocation
  geometry_msgs::msg::TransformStamped mps_transform;

  std::shared_ptr<MpsMapGenData> data_;
  std::unique_ptr<RefboxConnector> refbox_conn_;
  std::unique_ptr<WaitPosGen> wait_pos_gen_;
  bool use_refbox_data_ = true;
  bool publish_wait_pos_ = true;

  double approach_dist_ = 0.3;
  double border_thickness_ = 0.6;
};

} // namespace mps_map_gen

#endif
