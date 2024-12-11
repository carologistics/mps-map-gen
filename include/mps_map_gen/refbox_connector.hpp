// Copyright (c) 2024 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MPS_MAP_GEN_REFBOX_CONNECTOR_H
#define MPS_MAP_GEN_REFBOX_CONNECTOR_H

#include "mps.hpp"

#include <Eigen/Geometry>
#include <boost/asio.hpp>
#include <google/protobuf/message.h>
#include <protobuf_comm/message_register.h>
#include <protobuf_comm/peer.h>

#include <rclcpp/rclcpp.hpp>

namespace mps_map_gen {
class RefboxConnector {
public:
  RefboxConnector(const std::string &peer_address,
                  const unsigned short recv_port_public,
                  const unsigned short recv_port_magenta,
                  const unsigned short recv_port_cyan,
                  const std::string &team_name, const std::string &crypto_key,
                  const std::string &proto_path,
                  std::shared_ptr<MpsMapGenData> data,
                  const rclcpp::Logger &logger);

private:
  const rclcpp::Logger get_logger() { return logger_; };

  void handle_peer_msg(boost::asio::ip::udp::endpoint &endpoint,
                       uint16_t component_id, uint16_t msg_type,
                       std::shared_ptr<google::protobuf::Message> msg);
  void handle_peer_recv_error(boost::asio::ip::udp::endpoint &endpoint,
                              std::string msg);

  std::shared_ptr<std::vector<MPS>> mps_list_;

  std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> private_peer_;
  std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> public_peer_;
  std::shared_ptr<protobuf_comm::MessageRegister> message_register_;

  std::string peer_address_;
  std::string team_name_;
  std::string crypto_key_;
  unsigned short recv_port_magenta_;
  unsigned short recv_port_cyan_;
  enum TeamColor { NONE, CYAN, MAGENTA };
  TeamColor team_color_ = TeamColor::NONE;

  float get_x(std::string &zone_name);
  float get_y(std::string &zone_name);
  void mirror_mps(const MPS &mps);
  std::shared_ptr<MpsMapGenData> data_;
  const rclcpp::Logger logger_;
};

} // namespace mps_map_gen

#endif
