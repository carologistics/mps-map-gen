#include "mps_map_gen/mps_map_gen.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using std::placeholders::_1;
namespace mps_map_gen {
MpsMapGen::MpsMapGen() : Node("mps_map_gen"){
  map_client = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");
  mps_tf = this->create_subscription<tf2_msgs::msg::TFMessage>
    ("mps_tf", 10, std::bind(&MpsMapGen::mps_update, this, _1));
}

MpsMapGen::~MpsMapGen() {}


void MpsMapGen::mps_update(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg) {
  printf("update\n");
  // Wait for the service to become available
  while (!map_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
  }

  // Create a request object
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

  // Call the service
  auto future = map_client->async_send_request(request);

  // Wait for the response
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
      return;
  }
  auto response = future.get();
  printf("map\n");

  // Process the response data here
  // Example: Print the map resolution
  double resolution = response->map.info.resolution;
  int map_height = response->map.info.height;
  int map_width = response->map.info.width;

  //TODO ADD CFG
  double mps_length = 0.7;
  double mps_width = 0.35;
  for(const auto& transform : tf_msg->transforms) {
    double x = transform.transform.translation.x + response->map.info.origin.position.x;
    double y = transform.transform.translation.y + response->map.info.origin.position.y;
    //NEVER TODO handle map orientation. Who ever puts in a rotated map can add this themself
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(transform.transform.rotation, tf_quaternion);
    double rotation = tf2::getYaw(tf_quaternion);
    MPS mps(mps_length, mps_width, Eigen::Vector2f(x, y), Eigen::Rotation2Df(rotation));
    add_mps_to_map(mps, map_height, map_width, resolution, response->map.data);
  }
  printf("Hier\n");
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
  map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("ros2_map_updates", 10);

  // Publish the updated map
  map_publisher->publish(response->map);
}

void MpsMapGen::add_mps_to_map(MPS mps, int height, int width, double resolution, std::vector<int8_t> &data) {
  for (int i = 0; i < 4; ++i)
  {
    const Eigen::Vector2f &p1 = mps.corners[i];
    const Eigen::Vector2f &p2 = mps.corners[(i + 1) % 4];

    int x1 = static_cast<int>(p1.x() / resolution);
    int y1 = static_cast<int>(p1.y() / resolution);
    int x2 = static_cast<int>(p2.x() / resolution);
    int y2 = static_cast<int>(p2.y() / resolution);

    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
          if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height)
          {
            data[y1 * width + x1] = 1;
          }
          if (x1 == x2 && y1 == y2)
          {
            break;
          }
          int e2 = 2 * err;
          if (e2 > -dy)
          {
            err -= dy;
            x1 += sx;
          }
          if (e2 < dx)
          {
            err += dx;
            y1 += sy;
          }
    }
  }
}

}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mps_map_gen::MpsMapGen>());
  rclcpp::shutdown();
  return 0;
}
