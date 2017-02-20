-- publishers.t
--
-- various ros publishers

local m = {}
local class = require("class")
local math = require("math")

function m.resolve_topic(topic_value)
end

local Pose = class("Pose")
m.Pose = Pose

function Pose:init(ros, trackable, options)
  local topic_name = string.format(options.topic, trackable.device_idx)
  print("Creating new Pose publisher on " .. topic_name)
  self._topic = ros:topic({
    topicName = topic_name,
    messageType = "geometry_msgs/Pose",
    queueSize = options.queue_size or 10
  })
  self._decimate = options.decimate or 9 -- default to 10fps publishing
  self._position = math.Vector()
  self._quaternion = math.Quaternion()
  self._matrix = math.Matrix4()
  self._trackable = trackable
  self._frame = 0
end

function Pose:update()
  if not self._trackable.pose_valid then return end

  self._frame = self._frame + 1
  if (self._frame % self._decimate) ~= 0 then return end

  self._trackable.pose:get_column(4, self._position)
  self._trackable.pose:to_quaternion(self._quaternion)
  self._topic:publish({
    position    = self._position:to_dict3(),
    orientation = self._quaternion:to_dict()
  })
end

return m
