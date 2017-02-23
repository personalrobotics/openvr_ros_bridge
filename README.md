![HMD display and console output](screenshot.png?raw=true)

# openvr_ros_bridge
Publishes poses from HMD/controllers/anything trackable from OpenVR on Windows
to ROS over Rosbridge

# Prerequisites

* Windows: 64 bit windows and directx 11 support (e.g., a VR
  capable machine)
* ROS: the [rosbridge suite](http://wiki.ros.org/rosbridge_suite)

# Installation

Easy option: download the binary release and unzip somewhere convenient.

Harder option: compile the
[feature/ecs branch of truss](https://github.com/PyryM/truss/tree/feature/ecs),
zip all the folders in `truss/dist` into `truss.zip`, and place `truss.zip` and
`truss.exe` in the root of this repo.

# Usage

On a machine with ROS:

1. Start a roscore
2. Launch the ros web bridge: `roslaunch rosbridge_server rosbridge_websocket.launch`

On a windows machine with OpenVR and HMD connected:

1. Start SteamVR (optional, but greatly speeds up startup)
2. In a command prompt, navigate to the openvr_ros_bridge folder
3. In prompt run: `truss scripts/bridge.t "ws://[host]:[port]"`

# Configuration

The file `scripts/config.t` contains the configuration. By default, the HMD,
any controllers, and any generic trackables will have their poses published.
