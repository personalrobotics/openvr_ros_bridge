-- ros_publishers.t
--
--  ros-specific publishers

local m = {}
local publishers = require("publishers.t")

local Pose = publishers.Pose:extend("ROSPose")
m.Pose = Pose

function Pose:init(conn, trackable, options)
  Pose.super.init(self, conn, trackable, options)

  local topic_name = string.format(options.topic, trackable.device_idx)
  print("Creating new ROSPose publisher on " .. topic_name)
  self._dname = topic_name
  self._topic = conn:topic({
    topic_name = topic_name,
    message_type = "geometry_msgs/PoseStamped",
    queue_size = options.queue_size or 10
  })
  self._tf_frame = options.tf_frame or '0'
end

function Pose:publish()
  self._topic:publish({
    header = {frame_id = self._tf_frame},
    pose = {position = self._position:to_dict3(),
            orientation = self._quaternion:to_dict()}
    })
end

local ViveButtons = publishers.ViveButtons:extend("ROSViveButtons")
m.ViveButtons = ViveButtons

function ViveButtons:init(conn, trackable, options)
  ViveButtons.super.init(self, conn, trackable, options)

  local topic_name = string.format(options.topic, trackable.device_idx)
  print("Creating new ROSViveButtons publisher on " .. topic_name)
  self._dname = topic_name
  self._topic = conn:topic({
    topic_name = topic_name,
    message_type = "sensor_msgs/Joy",
    queue_size = options.queue_size or 10
  })
end

function ViveButtons:publish(msg)
  self._topic:publish(msg)
end

local ROSConnection = class("ROSConnection")
m.ROSConnection = ROSConnection
m.Connection = ROSConnection

function ROSConnection:init(url)
  local roslib = require("io/ros.t")
  self.url = url
  if url then
    self.ros = roslib.Ros()
    self.ros:connect(url)
  end
end

function ROSConnection:is_connected()
  return self.ros and self.ros.socket.open
end

function ROSConnection:status()
  if not self.url then
    return "ROS: No URL Specified"
  elseif self:is_connected() then
    return "ROS: " .. self.url
  else
    return "ROS: Disconnected"
  end
end

function ROSConnection:topic(opts)
  return self.ros:topic(opts)
end

function ROSConnection:update()
  if self.ros then
    self.ros:update()
  end
end

return m
