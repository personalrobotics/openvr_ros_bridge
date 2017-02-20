-- config.t
--
-- default openvr_ros_bridge configuration file

-- this file is loaded as a normal module, which means that it cannot create
-- global variables
local config = {}

config.Controller = {
  topic = "/openvr/controller_%d",
  publisher = "Pose"
}

config.Generic = {
  topic = "/openvr/generic_%d",
  publisher = "Pose"
}

config.HMD = {
  topic = "/openvr/hmd_%d",
  publisher = "Pose"
}

-- Don't publish anything for Reference class trackables
config.Reference  = nil

-- modules return themselves
return config
