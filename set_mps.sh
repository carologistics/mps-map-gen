#!/bin/sh
# Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

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
