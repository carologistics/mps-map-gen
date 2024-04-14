#ifndef MPS_MAP_GEN_REFBOX_CONNECTOR_H
#define MPS_MAP_GEN_REFBOX_CONNECTOR_H

#include "mps.hpp"

#include <Eigen/Geometry>
#include <boost/asio.hpp>
#include <google/protobuf/message.h>
#include <protobuf_comm/message_register.h>
#include <protobuf_comm/peer.h>

namespace mps_map_gen {
class RefboxConnector {
public:
  RefboxConnector(std::string &peer_address, unsigned short recv_port_public,
                  unsigned short recv_port_private, std::string &crypto_key,
                  std::string &proto_path, std::shared_ptr<MpsMapGenData> data);

private:
  void handle_peer_msg(boost::asio::ip::udp::endpoint &endpoint,
                       uint16_t component_id, uint16_t msg_type,
                       std::shared_ptr<google::protobuf::Message> msg);
  void handle_peer_recv_error(boost::asio::ip::udp::endpoint &endpoint,
                              std::string msg);

  std::shared_ptr<std::vector<MPS>> mps_list_;

  std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> private_peer_;
  std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer> public_peer_;
  std::shared_ptr<protobuf_comm::MessageRegister> message_register_;

  float get_x(std::string &zone_name);
  float get_y(std::string &zone_name);
  void mirror_mps(const MPS &mps);
  std::shared_ptr<MpsMapGenData> data_;
};

} // namespace mps_map_gen

#endif
