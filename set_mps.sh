#!/bin/sh

# Copyright (c) 2024 Carologistics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#base transform, identity quaternion
ros2 run tf2_ros static_transform_publisher 0 0 0 0.0 0.0 0.0 1.0 map mps &
#mps M-CS1 at (-5.5, 2.5, 0) with rotation 0   = (0.0000,0.0000,0.0000,1.0000)
ros2 run tf2_ros static_transform_publisher -5.5 2.5 0 0 0 0 1 mps M-CS1 &
#mps M-RS1 at (6.5, 6.5, 0) with rotation 90  = (0.0000,0.0000,0.7071,0.7071)
ros2 run tf2_ros static_transform_publisher 6.5 6.5 0 0 0 0.7071 0.7071 mps M-RS1 &
#mps M-BS  at (-3.5, 7.5, 0) with rotation 180 = (0.0000,0.0000,1.0000,0.0000)
ros2 run tf2_ros static_transform_publisher -3.5 7.5 0 0 0 1 0 mps M-BS &
#mps M-CS2 at (5.5, 4.5, 0) with rotation 45 = (0.0000,0.0000,-0.7071,0.7071)
ros2 run tf2_ros static_transform_publisher 5.5 4.5 0 0 0 0.3826 0.9238 mps M-CS2 && fg
