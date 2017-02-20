# openvr_ros_bridge
Publish from openvr/windows to ROS over rosbridge

# Installation

Obtain truss.exe and truss.zip from truss and place them in the root directory
(alongside this README.md).

# Running

On a linux machine with ROS installed (or through bash on windows), run the
rosbridge server via
`roslaunch rosbridge_server rosbridge_websocket.launch`

On the windows machine with OpenVR and whatever hardware you're using installed
(presumably an HTC vive), run
`truss scripts/bridge.t rosbridge_hostname [config.t]`
