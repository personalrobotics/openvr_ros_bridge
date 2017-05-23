-- config/ros.t
--
-- publish poses to ROS

-- so we can use the 'built in' publishers
local publishers = require("ros_publishers.t")
local vis = require("visualizers.t")

-- this file is loaded as a normal module, which means that it cannot create
-- global variables
local config = {}

-- set
config.Connection = publishers.ROSConnection

-- set up how different types of trackables are published
config.Controller = {
  topic = "/vr/controller_%d",
  publisher = publishers.Pose,
  decimate = 9, -- publish at approximately 10fps
  tf_frame = 'my_frame', -- defaults to '0'
  display = {vis.BasicModel(), vis.LineHistory()} -- show model and trails
}

config.Generic = {
  topic = "/vr/generic_%d",
  publisher = publishers.Pose,
  decimate = 9, -- publish at approximately 10fps
  display = {vis.BasicModel(), vis.LineHistory()} -- show model and trails
}

-- Don't publish anything for Reference or HMD class trackables
config.Reference  = nil
config.HMD        = nil

-- modules return themselves
return config
