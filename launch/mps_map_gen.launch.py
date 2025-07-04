#!/usr/bin/env python3
# Licensed under GPLv2+. See LICENSE file. Copyright Carologistics.
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the package share directory for rcll_protobuf_msgs
    rcll_protobuf_msgs_share_dir = get_package_share_directory("rcll_protobuf_msgs")

    # Define launch configuration variables
    get_data_from_refbox = LaunchConfiguration("get_data_from_refbox", default=True)
    namespace = LaunchConfiguration("namespace", default="")
    map_client = LaunchConfiguration("map_client", default="/map_server/map")
    publish_wait_pos = LaunchConfiguration("publish_wait_pos", default="true")
    peer_address = LaunchConfiguration("peer_address", default="127.0.0.1")
    recv_port_magenta = LaunchConfiguration("recv_port_magenta", default="4442")
    recv_port_cyan = LaunchConfiguration("recv_port_cyan", default="4441")
    recv_port_public = LaunchConfiguration("recv_port_public", default="4444")
    field_width = LaunchConfiguration("field_width", default="7")
    field_height = LaunchConfiguration("field_height", default="8")
    team_name = LaunchConfiguration("team_name", default="Carologistics")
    crypto_key = LaunchConfiguration("crypto_key", default="randomkey")
    # resolution = LaunchConfiguration('resolution', default=0.05)
    proto_path = LaunchConfiguration("proto_path", default=rcll_protobuf_msgs_share_dir + "/rcll-protobuf-msgs/")

    # Create the Node
    node = Node(
        package="mps_map_gen",
        executable="mps_map_gen",
        name="mps_map_gen",
        output="screen",
        parameters=[
            {"namespace": namespace},
            {"get_data_from_refbox": get_data_from_refbox},
            {"publish_wait_pos": publish_wait_pos},
            {"peer_address": peer_address},
            {"recv_port_magenta": recv_port_magenta},
            {"recv_port_cyan": recv_port_cyan},
            {"recv_port_public": recv_port_public},
            {"team_name": team_name},
            {"crypto_key": crypto_key},
            {"map_client": map_client},
            {"proto_path": proto_path},
            {"field_width": field_width},
            {"field_height": field_height},
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the parameters to the launch description
    ld.add_action(node)

    return ld
