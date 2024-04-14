#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2024
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for rcll_protobuf_msgs
    rcll_protobuf_msgs_share_dir = get_package_share_directory('rcll_protobuf_msgs')

    # Define launch configuration variables
    get_data_from_refbox = LaunchConfiguration('get_data_from_refbox', default=True)
    map_client = LaunchConfiguration('map_client', default="/map_server/map")
    publish_wait_pos = LaunchConfiguration('publish_wait_pos', default=True)
    peer_address = LaunchConfiguration('peer_address', default="127.0.0.1")
    recv_port_private = LaunchConfiguration('recv_port_private', default=4441)
    recv_port_public = LaunchConfiguration('recv_port_public', default=4444)
    crypto_key = LaunchConfiguration('crypto_key', default="randomkey")
    # resolution = LaunchConfiguration('resolution', default=0.05)
    proto_path = LaunchConfiguration('proto_path', default=rcll_protobuf_msgs_share_dir + '/rcll-protobuf-msgs/')

    # Create the Node
    node = Node(
        package='mps_map_gen',
        executable='mps_map_gen',
        name='mps_map_gen',
        output='screen',
        parameters=[
            {'get_data_from_refbox': get_data_from_refbox},
            {'publish_wait_pos': publish_wait_pos},
            {'peer_address': peer_address},
            {'recv_port_private': recv_port_private},
            {'recv_port_public': recv_port_public},
            {'crypto_key': crypto_key},
            {'map_client': map_client},
            {'proto_path': proto_path}
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the parameters to the launch description
    ld.add_action(node)

    return ld

