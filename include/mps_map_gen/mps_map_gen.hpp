#ifndef MPS_MAP_GEN_H
#define MPS_MAP_GEN_H

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "mps.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace mps_map_gen {
class MpsMapGen : public rclcpp::Node {
public:
  MpsMapGen();

  ~MpsMapGen();

  // TODO: CONFIG
  static constexpr double mps_length = 0.7;
  static constexpr double mps_width = 0.35;

private:
  void update_map();
  void add_mps_to_map(MPS mps, int height, int width, double resolution,
                      std::vector<int8_t> &data);
  void map_receive(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future);
  bool set_mps(MPS mps);
  void tfCallback();

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr mps_tf;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
      map_update_publisher;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pubsliher;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<MPS> mps_list;

  std::string mps_names[14] = {"M-BS",  "M-SS",  "M-DS",  "M-CS1", "M-CS2",
                               "M-RS1", "M-RS2", "C-BS",  "C-SS",  "C-DS",
                               "C-CS1", "C-CS2", "C-RS1", "C-RS2"};

  // only here to prevent realocation
  geometry_msgs::msg::TransformStamped mps_transform;

  bool needs_refresh = false;
};

} // namespace mps_map_gen

#endif
