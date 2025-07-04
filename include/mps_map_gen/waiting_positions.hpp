// Licensed under GPLv2+. See LICENSE file. Copyright Carologistics.

#ifndef MPS_MAP_GEN_WAITING_POSITIONS_H
#define MPS_MAP_GEN_WAITING_POSITIONS_H

#include "mps_map_gen/mps.hpp"

namespace mps_map_gen {
class WaitPosGen {
public:
  WaitPosGen(std::shared_ptr<MpsMapGenData> data);
  std::map<std::string, Eigen::Vector3f> generate_wait_pos(int num_free_zones);

private:
  std::shared_ptr<MpsMapGenData> data_;
  std::map<uint16_t, std::vector<Eigen::Vector2i>> zone_blocking_ = {
      {0,
       {
           // 0°
           {-1, 0}, //
           {1, 0}   // x|x
       }},
      {45,
       {         // 45°
        {1, 0},  //
        {1, 1},  //  xx
        {0, 1},  // x\x
        {0, -1}, // xx
        {-1, -1},
        {-1, 0}}},
      {90,
       {
           // 90°
           {0, 1}, //  x
           {0, -1} //  -
       }},         //  x
      {135,
       {{0, 1},     // 135°
        {-1, 1},    //
        {-1, 0},    // xx
        {1, 0},     // x/x
        {1, -1},    //  xx
        {0, -1}}}}; // 180°-315° is the same, see constructor.

  std::vector<Eigen::Vector2i> blocked_zones(const MPS &mps);
  inline uint16_t discrete_angle_from_yaw(float yaw);
  std::map<std::string, Eigen::Vector3f>
  generate_mps_wait_pos(std::vector<Eigen::Vector2i> &free_zones);
  Eigen::Vector2i point_to_zone(const Eigen::Vector2f &p);
  Eigen::Vector2f zone_to_point(const Eigen::Vector2i &p);
  std::map<std::string, Eigen::Vector3f>
  generate_free_wait_pos(const std::vector<std::vector<bool>> &occupancy_grid,
                         int num_points);
};

} // namespace mps_map_gen

#endif
