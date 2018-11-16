-- config/default.t
--
-- default openvr_ros_bridge configuration file that writes to a file

-- so we can use the 'built in' publishers
local publishers = require("file_publishers.t")
local vis = require("visualizers.t")

-- this file is loaded as a normal module, which means that it cannot create
-- global variables
local config = {}

-- set
config.Connection = publishers.FileConnection

-- set up how different types of trackables are published
config.Controller = {
  topic = "controller_%d",
  publisher = publishers.Pose,
  decimate = 1, -- no decimation, write the full 90 fps
  -- field_order = {"time", "position", "quaternion", "velocity", "angular_velocity"},
  display = {
    vis.BasicModel{
      material = {diffuse = {1.0, 0.0, 0.0, 1.0}}
    },
    vis.LineHistory()
  } 
}

config.Generic = {
  topic = "generic_%d",
  publisher = publishers.Pose,
  decimate = 1, -- no decimation, write the full 90 fps
  display = {vis.BasicModel(), vis.LineHistory()} -- show model and trails
}

-- Don't publish anything for Reference or HMD class trackables
config.Reference  = nil
config.HMD        = nil

-- modules return themselves
return config
