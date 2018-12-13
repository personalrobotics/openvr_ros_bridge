-- config/default.t
--
-- default openvr_ros_bridge configuration file that writes to a file

-- so we can use the 'built in' publishers
local publishers = require("file_publishers.t")
local vis = require("visualizers.t")

-- this file is loaded as a normal module, which means that it cannot create
-- global variables
local config = {}

-- set dimensions of display window (default: 1280x720)
config.width  = 1280
config.height = 720

-- whether to display stats (default: true)
config.stats = true

-- the pose polling rate can be set (defaults to 60hz, max ~200hz)
-- at 60hz it will run with vsync, so the timing will be fairly consistent
-- at other rates, it tries to estimate how long it needs to sleep between
-- polls, which results in less consistent timing
config.rate = 60

-- set up where data is published (to a file in this case)
config.Connection = publishers.FileConnection

-- set up how different types of trackables are published
config.Controller = {
  topic = "controller_%d",
  publisher = publishers.Pose,
  decimate = 1, -- no decimation, write at full polling rate
  -- field_order = {"time", "position", "quaternion", "velocity", "angular_velocity"},
  display = {
    vis.BasicModel{
      material = {diffuse = {1.0, 0.0, 0.0, 1.0}}
    },
    vis.LineHistory(),
    vis.PropDisplay{prop = "time", font_size = 80, format = "%0.5f"}
  } 
}

config.Generic = {
  topic = "generic_%d",
  publisher = publishers.Pose,
  decimate = 1, -- no decimation, write at full polling rate
  display = {vis.BasicModel(), vis.LineHistory()} -- show model and trails
}

-- Don't publish anything for Reference or HMD class trackables
config.Reference  = nil
config.HMD        = nil

-- modules return themselves
return config
