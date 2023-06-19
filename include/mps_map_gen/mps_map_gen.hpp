#ifndef MPS_MAP_GEN_H
#define MPS_MAP_GEN_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include <Eigen/Geometry>
namespace mps_map_gen {
class MPS
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MPS(double mps_length, double mps_width, Eigen::Vector2f center, Eigen::Rotation2Df rot){
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
    Eigen::Vector2f corners[4];
    Eigen::Vector2f center_;
    float angle;
};

class MpsMapGen : public rclcpp::Node {
public:
    MpsMapGen();

    ~MpsMapGen();
private:
    void mps_update(const tf2_msgs::msg::TFMessage::SharedPtr tf_msg);
    void add_mps_to_map(MPS mps, int height, int width, double resolution, std::vector<int8_t> &data);
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr mps_tf;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client;
    rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_publisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map2_publisher;
};
}

#endif
