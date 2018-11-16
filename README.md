![HMD display and console output](screenshot.png?raw=true)

# openvr_ros_bridge
Publishes poses from HMD/controllers/anything trackable from OpenVR on Windows
to ROS. It can also publish JSON to a generic websocket target, or log poses
to a file.

# Prerequisites

* Windows: 64 bit windows and directx 11 support (e.g., a VR
  capable machine)
* Linux: 64 bit Linux with real graphics drivers (NOT MESA)
* [optional] ROS: the [rosbridge suite](http://wiki.ros.org/rosbridge_suite)

# Installation

Windows: download [the binary release](https://github.com/personalrobotics/openvr_ros_bridge/releases/tag/v0.3.0)
and unzip somewhere convenient.

Linux: download the binary release. Then compile the `renderimprovements` 
branch of [truss](https://github.com/PyryM/truss).

E.g.,
```
git clone --single-branch -b renderimprovements https://github.com/PyryM/truss 
mkdir build
cd build
cmake ..
make
```

Then from `dist/` copy the truss binary `truss` and the `libs/` folder
 (hopefully populated with .so files) into the directory where you extracted
 `openvr_ros_bridge`.

# Usage

## General / File logging

SteamVR should be started before the bridge, otherwise the bridge will itself
try to start SteamVR, which is unreliable and may lead to a freeze or a crash.

`truss url_or_filename config_script_name.t`

For example, the config script `config/default.t` will log to a text file in
csv format. To log to "my_log.txt", run

`truss my_log.txt default.t`

## Running without an HMD

It is possible to run without the HMD connected, if you have Vive trackers,
or if you have paired the controllers to a Bluetooth adaptor (normally the
controllers pair to the built-in Bluetooth of the HMD, which is why it must
be plugged in).

The [steamvr.vrsettings file](https://developer.valvesoftware.com/wiki/SteamVR/steamvr.vrsettings)
must be edited so that
```
"requireHmd": false,
```

## AMD Ryzen note

If you have an AMD Ryzen or other exotic CPU, you will probably need to edit
`scripts/core/core.t` within `truss.zip` and change line 8 to:
```lua
local use_ryzen_hack = true
```

## Linux notes

If you see the message `Going to create window; if you get an LLVM crash on linux at this point, the mostly likely reason is that you are using the mesa software renderer.` and then truss crashes, then you are probably using Mesa
drivers. The solution is to install real drivers. You can also try to disable
Mesa/llvmpipe by setting the `DRAW_USE_LLVM` environment variable to 0.

SteamVR has special requirements to run on Linux: see 
[the SteamVR Linux readme](https://github.com/ValveSoftware/SteamVR-for-Linux/blob/master/README.md).
In particular, you will need to invoke truss like
```
~/.steam/steam/ubuntu12_32/steam-runtime/run.sh ./truss [bridge options]
```

## ROS

On a machine with ROS:

1. Start a roscore
2. Launch the ros web bridge: `roslaunch rosbridge_server rosbridge_websocket.launch`

On the VR machine (can be the same as the ROS machine):

1. Start SteamVR (optional, but greatly speeds up startup)
2. In a command prompt/terminal, navigate to the openvr_ros_bridge folder
3. In prompt run: `truss "ws://[host]:[port]" ros.t`. Note that the
default rosbridge port is 9090, so this will look something like
`truss "ws://othercomp:9090" ros.t`. (For Linux, see the note above regarding
running with the Steam runtime).

# Configuration

Config scripts should be placed in `scripts/config/`.
The file `scripts/config/default.t` contains the default configuration that will
log poses to a file in CSV format. By default only controllers and generic
trackables are logged.

## File logging configuration

```lua
local publishers = require("file_publishers.t")
local config = {}
config.Connection = publishers.FileConnection
config.Controller = {
  topic = "controller_%d",
  publisher = publishers.Pose,
  display = true
}
return config
```

The file Pose publisher accepts the following additional options

| Option        | Description           | Default  |
| ------------- |---------------------- | -------- |
| decimate      | Publish only 1 in n frames | 9 |
| field_order   | Ordered list of fields to write to CSV | {"time", "position", "quaternion"} |
| divider       | Field divider      | "," |
| precision     | Numerical precision to write | 4 |
| format        | Numerical format string (overrides precision) | "%.4f" |

## ROS publishing configuration

```lua
local publishers = require("ros_publishers.t")
local config = {}
config.Connection = publishers.ROSConnection
config.Controller = {
  topic = "/vr/controller_%d",
  publisher = publishers.Pose,
  display = true
}
return config
```

The ROS Pose publisher accepts the following additional options

| Option        | Description           | Default  |
| ------------- |---------------------- | -------- |
| decimate      | Publish only 1 in n frames | 9 |
| tf_frame   | ROS TF frame to associate PoseStamped messages with | "0" |
| queue_size       | Maximum ROS queue size  | 10 |


## Visualization configuration

Visualization options are independent of publishing/logging, and are specified
in the `display` options in a configuration. Setting `display = true` uses the
default visualization option, which is to just show the model. Likewise,
`display = false` will not display anything (but will still publish if publication
options are set).

Additional visualization options can be included by setting `display`.
```lua
local vis = require("visualizers.t")

config.Controller = {
  -- [...]
  display = {vis.BasicModel(), vis.LineHistory()} -- show model and trails
}
```

Note the slightly unconventional syntax of having to actually call/construct
the visualizer options, which is to allow additional options to be passed to
a specific visualizer.

### LineHistory
Adds a trail to a trackable.

```lua
vis.LineHistory({
  history_length = 900,    -- how many points the line can have
  decimate = 3,            -- only push a new point every n frames
  color = {0.8, 0.3, 0.3}, -- line color, in [0,1] RGB format
  thickness = 0.005        -- line thickness, in m
})
```

### BasicModel
Shows a mesh model for the trackable, loaded from OpenVR.

```lua
vis.BasicModel({
  diffuse = {0.03,0.03,0.03,1.0},   -- diffuse color
  specular = {0.001, 0.001, 0.001}, -- specular color
  roughness = 0.7                   -- roughness
})
```
