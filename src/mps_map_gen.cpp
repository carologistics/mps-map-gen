#include "mps_map_gen/mps_map_gen.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using std::placeholders::_1;
namespace mps_map_gen {
MpsMapGen::MpsMapGen() : Node("mps_map_gen"){
  map_client = this->create_client<nav_msgs::srv::GetMap>("map_server/map");
  mps_tf = this->create_subscription<tf2_msgs::msg::TFMessage>
    ("mps_tf", 10, std::bind(&MpsMapGen::mps_update, this, _1));

  // Set QoS to match the expected settings for map visualization in RViz
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.history(rclcpp::HistoryPolicy::KeepLast);


  map_publisher = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>("ros2_map_updates", 10);
  map2_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("tim_map", qos);
  RCLCPP_INFO(this->get_logger(), "Start");
}

map_msgs::msg::OccupancyGridUpdate convertOccupancyGridToOccupancyGridUpdate(const nav_msgs::msg::OccupancyGrid& occupancy_grid)
{
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


void MpsMapGen::mps_update(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg) {
  RCLCPP_INFO(this->get_logger(), "Update");
  // Wait for the service to become available
  while (!map_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
  }
  RCLCPP_INFO(this->get_logger(), "Client");

  // Create a request object
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  RCLCPP_INFO(this->get_logger(), "Request");

// Call the service
auto result_future = map_client->async_send_request(request,
    [this, tf_msg](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "map");

            double resolution = response->map.info.resolution;
            int map_height = response->map.info.height;
            int map_width = response->map.info.width;

            double mps_length = 0.7;
            double mps_width = 0.35;

            for (const auto &transform : tf_msg->transforms)
            {
                double x = transform.transform.translation.x - response->map.info.origin.position.x;
                double y = transform.transform.translation.y - response->map.info.origin.position.y;
                RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f", x, y);
                tf2::Quaternion tf_quaternion;
                tf2::fromMsg(transform.transform.rotation, tf_quaternion);
                double rotation = tf2::getYaw(tf_quaternion);
                MPS mps(mps_length, mps_width, Eigen::Vector2f(x, y), Eigen::Rotation2Df(rotation));
                add_mps_to_map(mps, map_height, map_width, resolution, response->map.data);
            }
            // for(unsigned int i = 0; i < 1000000; i++) {
            //   response->map.data[i] = 100;
            // }
            RCLCPP_INFO(this->get_logger(), "hier");

            // Publish the updated map
            map_publisher->publish(convertOccupancyGridToOccupancyGridUpdate(response->map));
            map2_publisher->publish(response->map);
            RCLCPP_INFO(this->get_logger(), "published");

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service: %s", e.what());
        }
    });


  // Call the service
  // auto future = map_client->async_send_request(request);
  // RCLCPP_INFO(this->get_logger(), "answer");

  //       // Create a single-threaded executor
  //       rclcpp::executors::SingleThreadedExecutor executor;

  //     executor.spin_until_future_complete(future);
  //       // Use spin_until_future_complete with the executor
  //     // if (executor.spin_until_future_complete(future) 
  //     //     == rclcpp::FutureReturnCode::SUCCESS)
  //     // {
  //     //     RCLCPP_ERROR(this->get_logger(), "Failed to call service");
  //     //     return;
  //     // }
  //     RCLCPP_INFO(this->get_logger(), "future");
  //     auto response = future.get();
  //     RCLCPP_INFO(this->get_logger(), "map");

  //     // Process the response data here
  //     // Example: Print the map resolution
  //     double resolution = response->map.info.resolution;
  //     int map_height = response->map.info.height;
  //     int map_width = response->map.info.width;

  //     // TODO ADD CFG
  //     double mps_length = 0.7;
  //     double mps_width = 0.35;
  //     for (const auto &transform : tf_msg->transforms)
  //     {
  //         double x = transform.transform.translation.x + response->map.info.origin.position.x;
  //         double y = transform.transform.translation.y + response->map.info.origin.position.y;
  //         // NEVER TODO handle map orientation. Who ever puts in a rotated map can add this themself
  //         tf2::Quaternion tf_quaternion;
  //         tf2::fromMsg(transform.transform.rotation, tf_quaternion);
  //         double rotation = tf2::getYaw(tf_quaternion);
  //         MPS mps(mps_length, mps_width, Eigen::Vector2f(x, y), Eigen::Rotation2Df(rotation));
  //         add_mps_to_map(mps, map_height, map_width, resolution, response->map.data);
  //     }
  //     RCLCPP_INFO(this->get_logger(), "hier");
  //     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
  //     map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("ros2_map_updates", 10);

  //     // Publish the updated map
  //     map_publisher->publish(response->map);
  }

void MpsMapGen::add_mps_to_map(MPS mps, int height, int width, double resolution, std::vector<int8_t> &data) {

    // int up_left_x = static_cast<int>(mps.up_left.x() / resolution);
    // int up_left_y = static_cast<int>(mps.up_left.y() / resolution);
    // int up_right_x = static_cast<int>(mps.up_right.x() / resolution);
    // int up_right_y = static_cast<int>(mps.up_right.y() / resolution);
    // int down_left_x = static_cast<int>(mps.down_left.x() / resolution);
    // int down_left_y = static_cast<int>(mps.down_left.y() / resolution);
    // int down_right_x = static_cast<int>(mps.down_right.x() / resolution);
    // int down_right_y = static_cast<int>(mps.down_right.y() / resolution);

    // int dx = std::abs(up_left_x - up_right_x);
    // int dy = std::abs(up_left_y - down_left_y);
    // int sx = (up_left_x < up_left_x) ? 1 : -1;
    // int sy = (up_left_y < down_left_y) ? 1 : -1;
    // int err = dx - dy;

    // for(int y = down_left_y; y < up_left_y && y < height; ++y) {
    //   for(int x = up_left_x; x < up_right_x && x < width; ++x) {
    //     data[y * width + x] = 100;
    //   }
    // }

    // Calculate the bounding box of the rotated box
    int minX = std::min({static_cast<int>(mps.corners[0].x() / resolution), static_cast<int>(mps.corners[1].x() / resolution), static_cast<int>(mps.corners[2].x() / resolution), static_cast<int>(mps.corners[3].x() / resolution)});
    int minY = std::min({static_cast<int>(mps.corners[0].y() / resolution), static_cast<int>(mps.corners[1].y() / resolution), static_cast<int>(mps.corners[2].y() / resolution), static_cast<int>(mps.corners[3].y() / resolution)});
    int maxX = std::max({static_cast<int>(mps.corners[0].x() / resolution), static_cast<int>(mps.corners[1].x() / resolution), static_cast<int>(mps.corners[2].x() / resolution), static_cast<int>(mps.corners[3].x() / resolution)});
    int maxY = std::max({static_cast<int>(mps.corners[0].y() / resolution), static_cast<int>(mps.corners[1].y() / resolution), static_cast<int>(mps.corners[2].y() / resolution), static_cast<int>(mps.corners[3].y() / resolution)});

    float cosAngle = std::cos(mps.angle);
    float sinAngle = std::sin(mps.angle);
    // Clamp the bounding box to image boundaries
    minX = std::max(0, minX);
    minY = std::max(0, minY);
    maxX = std::min(width - 1, maxX);
    maxY = std::min(height - 1, maxY);

    // Iterate over the pixels within the bounding box
    for (int py = minY; py <= maxY; ++py)
    {
        for (int px = minX; px <= maxX; ++px)
        {
          //width and height to confgi

            double mps_length = 0.7;
            double mps_width = 0.35;
            int mps_x = static_cast<int>(mps.center_.x() / resolution);
            int mps_y = static_cast<int>(mps.center_.y() / resolution);
            int cx = px - mps_x;
            int cy = py - mps_y;

            // Check if the pixel is inside the rotated box
            if (std::abs(cx * cosAngle + cy * sinAngle) <= mps_width / resolution / 2 && std::abs(-cx * sinAngle + cy * cosAngle) <= mps_length / resolution / 2)
            {
              data[py * width + px] = 100;
            }
        }
    }
    // while (true)
    // {
    //       if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height)
    //       {
    //         data[y1 * width + x1] = 100;
    //       }
    //       if (x1 == x2 && y1 == y2)
    //       {
    //         break;
    //       }
    //       int e2 = 2 * err;
    //       if (e2 > -dy)
    //       {
    //         err -= dy;
    //         x1 += sx;
    //       }
    //       if (e2 < dx)
    //       {
    //         err += dx;
    //         y1 += sy;
    //       }
    // }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mps_map_gen::MpsMapGen>());
  rclcpp::shutdown();
  return 0;
}
