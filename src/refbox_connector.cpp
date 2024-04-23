#include "rcll_protobuf_cpp/GameState.pb.h"
#include "rcll_protobuf_cpp/MachineInfo.pb.h"
#include "rcll_protobuf_cpp/VersionInfo.pb.h"

#include "mps_map_gen/refbox_connector.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
namespace mps_map_gen {
RefboxConnector::RefboxConnector(
    const std::string &peer_address, const unsigned short recv_port_public,
    const unsigned short recv_port_magenta, const unsigned short recv_port_cyan,
    const std::string &team_name, const std::string &crypto_key,
    const std::string &proto_path, std::shared_ptr<MpsMapGenData> data,
    const rclcpp::Logger &logger)
    : peer_address_(peer_address), team_name_(team_name),
      crypto_key_(crypto_key), recv_port_magenta_(recv_port_magenta),
      recv_port_cyan_(recv_port_cyan), logger_(logger) {
  data_ = data;
  // setup protobuf communication
  std::vector<std::string> proto_path_vec = {proto_path};
  message_register_ =
      std::make_shared<protobuf_comm::MessageRegister>(proto_path_vec);

  private_peer_->signal_received().connect(
      boost::bind(&RefboxConnector::handle_peer_msg, this,
                  boost::placeholders::_1, boost::placeholders::_2,
                  boost::placeholders::_3, boost::placeholders::_4));
  private_peer_->signal_recv_error().connect(
      boost::bind(&RefboxConnector::handle_peer_recv_error, this,
                  boost::placeholders::_1, boost::placeholders::_2));
  public_peer_ = std::make_shared<protobuf_comm::ProtobufBroadcastPeer>(
      peer_address, recv_port_public, message_register_.get());

  public_peer_->signal_received().connect(
      boost::bind(&RefboxConnector::handle_peer_msg, this,
                  boost::placeholders::_1, boost::placeholders::_2,
                  boost::placeholders::_3, boost::placeholders::_4));
  public_peer_->signal_recv_error().connect(
      boost::bind(&RefboxConnector::handle_peer_recv_error, this,
                  boost::placeholders::_1, boost::placeholders::_2));
}

void RefboxConnector::handle_peer_recv_error(boost::asio::ip::udp::endpoint &,
                                             std::string) {}

float RefboxConnector::get_x(std::string &zone_name) {
  float x = 0;
  char first_char = zone_name[0];
  float first_digit = zone_name[3] - '0';

  // Determine the sign based on the first character
  float sign = (first_char == 'C') ? 1 : -1;

  // Calculate the x-coordinate of center
  x = sign * (first_digit - 0.5);

  return x;
}
float RefboxConnector::get_y(std::string &zone_name) {
  // Calculate y-coordinate of center
  return zone_name[4] - '0' - 0.5;
}

void RefboxConnector::mirror_mps(const MPS &mps) {
  if (data_->field_width < 1 || data_->field_height < 1) {
    return;
  }
  if (mps.name_.length() > 0) {
    std::string mirrored_name = mps.name_;
    if (mirrored_name[0] == 'C') {
      mirrored_name[0] = 'M';
    } else {
      mirrored_name[0] = 'C';
    }
    float threshold = 0.01f;
    float mirrored_rot = 0;
    if ((mps.angle < threshold) || (std::abs(mps.angle - M_PI) < threshold)) {
      mirrored_rot = mps.angle + M_PI;
    } else if (std::abs(mps.angle - 45 * M_PI / 180.) < threshold ||
               std::abs(mps.angle - 135 * M_PI / 180.) < threshold) {
      mirrored_rot = mps.angle + M_PI / 2.;
    } else if (std::abs(mps.angle - 90 * M_PI / 180.) < threshold ||
               std::abs(mps.angle - 270 * M_PI / 180.) < threshold) {
      mirrored_rot = mps.angle;
    } else {
      fflush(stdout);
    }

    if (std::abs(mps.center_[0]) + 1 > data_->field_width ||
        std::abs(mps.center_[1]) + 1 > data_->field_height ||
        mps.center_[1] - 1 < 0 ||
        (((data_->field_width - 3) < std::abs(mps.center_[0])) &&
         std::abs(mps.center_[1] < 2))) {
      if (mirrored_name.find("RS") != std::string::npos ||
          mirrored_name.find("CS") != std::string::npos) {
        mirrored_rot = mps.angle;
      }
    }
    MPS new_mps(Eigen::Vector2f(-mps.center_[0], mps.center_[1]),
                Eigen::Rotation2Df(mirrored_rot), mirrored_name);
    { data_->set_mps(new_mps); }
  }
}

void RefboxConnector::handle_peer_msg(
    boost::asio::ip::udp::endpoint &, uint16_t, uint16_t,
    std::shared_ptr<google::protobuf::Message> msg_ptr) {
  if (!msg_ptr) {
    RCLCPP_DEBUG(get_logger(), "Received invalid msg ptr");
    return;
  }
  const google::protobuf::Descriptor *desc = msg_ptr->GetDescriptor();
  // Process the message based on its type
  if (desc->name() == "MachineInfo") {
    const llsf_msgs::MachineInfo *machine_info_msg =
        dynamic_cast<const llsf_msgs::MachineInfo *>(msg_ptr.get());
    if (machine_info_msg) {

      for (const auto &machine : machine_info_msg->machines()) {
        if (machine.has_rotation() && machine.has_zone()) {
          // Rotation field is set, retrieve its value
          std::string zone_name = llsf_msgs::Zone_Name(machine.zone());
          // Calculate x, y coordinates
          float x = get_x(zone_name);
          float y = get_y(zone_name);
          MPS new_mps(Eigen::Vector2f(x, y),
                      Eigen::Rotation2Df(machine.rotation() * M_PI / 180.0),
                      machine.name());
          { data_->set_mps(new_mps); }
          { mirror_mps(new_mps); }
        }
      }
    }
  } else if (desc->name() == "VersionInfo") {
    const llsf_msgs::VersionInfo *version_info_msg =
        dynamic_cast<const llsf_msgs::VersionInfo *>(msg_ptr.get());
    if (version_info_msg) {
      for (const auto &conf : version_info_msg->configuration()) {
        if (conf.name() == "field_width" &&
            static_cast<uint32_t>(data_->field_width) != conf.integer_value()) {
          RCLCPP_DEBUG(this->get_logger(), "Setting field width to %i",
                       conf.integer_value());
          std::lock_guard<std::mutex> lock(data_->data_mutex);
          data_->field_width = conf.integer_value();
          data_->needs_refresh = true;
        }
        if (conf.name() == "field_height" &&
            static_cast<uint32_t>(data_->field_height) !=
                conf.integer_value()) {
          RCLCPP_DEBUG(this->get_logger(), "Setting field height to %i",
                       conf.integer_value());
          std::lock_guard<std::mutex> lock(data_->data_mutex);
          data_->field_height = conf.integer_value();
          data_->needs_refresh = true;
        }
        if (conf.name() == "field_mirrored" &&
            conf.boolean_value() != data_->field_mirrored) {
          RCLCPP_DEBUG(this->get_logger(), "Setting field mirrored to %s",
                       conf.boolean_value() ? "true" : "false");
          std::lock_guard<std::mutex> lock(data_->data_mutex);
          data_->field_mirrored = conf.boolean_value();
          data_->needs_refresh = true;
        }
      }
    }
  } else if (desc->name() == "GameState") {
    const llsf_msgs::GameState *game_state_msg =
        dynamic_cast<const llsf_msgs::GameState *>(msg_ptr.get());
    if (game_state_msg->team_cyan() == team_name_) {
      RCLCPP_INFO(this->get_logger(),
                  "Listening to CYAN peer %s:%i for team %s",
                  peer_address_.c_str(), recv_port_cyan_, team_name_.c_str());
      private_peer_ = std::make_shared<protobuf_comm::ProtobufBroadcastPeer>(
          peer_address_, recv_port_cyan_, message_register_.get(), crypto_key_);
    } else if (game_state_msg->team_magenta() == team_name_) {
      RCLCPP_INFO(
          this->get_logger(), "Listening to MAGENTA peer %s:%i for team %s",
          peer_address_.c_str(), recv_port_magenta_, team_name_.c_str());
      private_peer_ = std::make_shared<protobuf_comm::ProtobufBroadcastPeer>(
          peer_address_, recv_port_magenta_, message_register_.get(),
          crypto_key_);
    } else if (!private_peer_) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for RefBox to send information on team %s",
                  team_name_.c_str());
    }
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Received %s", desc->name().c_str());
  }
}

} // namespace mps_map_gen
