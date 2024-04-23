# mps-map-gen
ROS 2 node to extend maps published by an existing map server by game-specific information from an instance of the RoboCup Logistics League field, such as machines placed on the field and boundaries for the competition area.

It provides two maps:
1) mps_map: contains just the original map + the machines drawn by this node, which can be used as input to the localization component (such as amcl).
2) mps_map_bounded: contains mps_map + a bounding box for the legal field, which can be used for navigation components to prevent robots from driving outside of the allowed game area.

## Installation

All dependencies for this repository that are not pre-packaged for Ubuntu already are maintained in the `dependencies.repos` file.

### Workspace Setup

After installing ROS2, create a fresh workspace and clone this repository and it's dependencies:
```bash
mkdir -p ~/ros2/mps_map_gen_ws/src
cd ~/ros2/mps_map_gen_ws/src
git clone https://github.com/carologistics/mps-map-gen.git
vcs import --recursive < ~/ros2/mps_map_gen_ws/src/mps-map-gen/dependencies.repos
```

### Building

```bash
cd ~/ros2/mps_maps_gen_ws
colcon build --symlink-install
```

### Launch mps_map_gen

```bash
ros2 launch mps_map_gen mps_map_gen.launch.py
```

- namespace: namespace for the node
- get_data_from_refbox: if true, listen to refbox to get data, otherwise listen to tf tree.
  - peer_address: address where the refbox broadcasts to
  - recv_port_magenta: port of team magenta, where the refbox sends information to
  - recv_port_cyan: port of team cyan, where the refbox sends information to
  - recv_port_public: public send port of the refbox
  - team_name: name of the team to publish map information to
  - crypto_key: crypto key of the team (used to decrypt information send over team peer)
  - proto_path: path to the protobuf msg definitions (defaults to the messages provided by [the rcll-protobuf](https://github.com/carologistics/rcll-protobuf) ROS2 helper packages)
- publish_wait_pos: if true, publish useful waiting positions
- map_client: name of the map client to connect to
- field_width: initial field width in meters (may be overridden by information from the refbox)
- field_height: initial field height in meters (may be overridden by information from the refbox)

## Operation Modes
It can operate in two different modes:
1) The node listens to the broadcasted protobuf messages of a running [refbox](https://github.com/robocup-logistics/rcll-refbox) to determine the field dimensions, the played team color (given a team name) and the ground-truth information of machines sent to that team.
It additionally publishes static transforms (relative to the map frame) of the machines (e.g., named `M-BS`) with the x-axis pointing towards the machine's inpt side, as well as transforms in front and behind the machine that point towards it (e.g., `M-BS-INPUT` and `M-BS-OUTPUT`).
These transforms can serve as useful navigation targets for robots that need to operate the machines.
In this mode machines of the opposing teams are also mirrored according to the rules of the game.
2) The other option is to run independently from the refbox and instead accept configurable values for field dimensions as well as static transforms describing the machine positions.
For example, when it receives a transform named `M-BS`, it then transforms it to map frame before drawing an obstacle around it treating the transformed point as the center of a machine, and the rotation of the pose as description of the conveyor belt direction (the belt is following the x-axis of the target pose from machine output to input). The usage of this mode is recommended in cases where some low-level routines should be tested without a running refbox.

In both modes it requires the map to be aligned with the game's coordinate system (The x-dimension moves from left to right along the columns of the occupancy grid, while the y-dimension moves from bottom to top along the rows of the occupancy grid).
In particular, **this tool does not support drawing over map servers that represent a rotated view of the game field**.

## Waiting Position Generation
Additionally, this tool also supports the generation of static transforms that can be useful when navigating on a RCLL field.
It offers two kind of waiting positions:
1) Firstly, it can generate positions that are close to a machines input or output side, while not being in any of the zones needed to operate the machines (see the blocked zones in the [rulebook](https://github.com/robocup-logistics/rcll-rulebook).
For example, `WAIT-C-BS-INPUT` would be the closest point to the input side of the machine named `C-BS`, with an orientation facing towards that machine.
This is useful if robots want to start heading towards a machine that is currently being used. Note that waiting positions of different machines may overlap (e.g., it is possible for `WAIT-C-BS-INPUT` and `WAIT-C-CS1-OUTPUT` to have the same x and y coordinates).
Hence, care must be taken when coordinating multiple robots among the waiting positions (e.g., allow high tolerances regarding the deviation between the target goal position and the actual position).

2) Secondly, 4 Waiting positions will be generated that are as far away from any machine as possible, called `WAIT1` to `WAIT4`, while not overlapping among each other. These points are useful targets in case a robot is not needed at the moment and should not block any important target.
Note that it is possible for these kind of waiting positions to overlap with waiting positions from the first category (e.g., `WAIT-C-BS-INPUT` might have the same x and y coordinates as `WAIT4`).
