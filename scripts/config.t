-- config.t
--
-- default openvr_ros_bridge configuration file

-- so we can use the 'built in' publishers
local publishers = require("publishers.t")

-- this file is loaded as a normal module, which means that it cannot create
-- global variables
local config = {}

-- set up how different types of trackables are published
config.Controller = {
  topic = "/openvr/controller_%d",
  publisher = publishers.Pose
  --          publishers.ViveButtons -- if you want to publish the buttons
}

config.Generic = {
  topic = "/openvr/generic_%d",
  publisher = publishers.Pose
}

config.HMD = {
  topic = "/openvr/hmd_%d",
  publisher = publishers.Pose
}

-- Don't publish anything for Reference class trackables
config.Reference  = nil

-- modules return themselves
return config
