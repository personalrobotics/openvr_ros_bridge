# openvr_ros_bridge
Publish from openvr/windows to ROS over rosbridge

# Installation

Obtain truss.exe and truss.zip from truss and place them in the root directory
(alongside this README.md).

# Running

On a machine with ROS:
`roslaunch rosbridge_server rosbridge_websocket.launch`

On the windows machine with OpenVR and HMD:
[strongly recommended] start SteamVR
`truss scripts/bridge.t rosbridge_hostname [config.t]`
