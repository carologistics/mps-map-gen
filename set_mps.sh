#!/bin/bash
ros2 topic pub /mps_tf tf2_msgs/TFMessage \
'{transforms: [
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "frame1"}, child_frame_id: "frame2", transform: {translation: {x: -0.5, y: 2.5, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}},
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "frame1"}, child_frame_id: "frame2", transform: {translation: {x: -1.5, y: 2.5, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}},
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "frame1"}, child_frame_id: "frame2", transform: {translation: {x: -3.5, y: 1.5, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0, w: 1}}},
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "frame1"}, child_frame_id: "frame2", transform: {translation: {x: -3.5, y: 2.5, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0, w: 1}}},
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "frame1"}, child_frame_id: "frame2", transform: {translation: {x: -3.5, y: 4.5, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}},
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "frame1"}, child_frame_id: "frame2", transform: {translation: {x: -4.5, y: 4.5, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}},
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "frame1"}, child_frame_id: "frame2", transform: {translation: {x: -1.5, y: 4.5, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1}}}
]}' \
-r 1 --once
