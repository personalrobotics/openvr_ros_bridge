-- publishers.t
--
-- various ros publishers

local m = {}
local class = require("class")
local math = require("math")

local Pose = class("Pose")
m.Pose = Pose

function Pose:init(conn, trackable, options)
  local topic_name = string.format(options.topic, trackable.device_idx)
  print("Creating new Pose publisher on " .. topic_name)
  self._tname = topic_name
  self._topic = conn:topic({
    topicName = topic_name,
    messageType = "geometry_msgs/PoseStamped",
    queueSize = options.queue_size or 10
  })
  self._decimate = options.decimate or 9 -- default to 10fps publishing
  self._position = math.Vector()
  self._quaternion = math.Quaternion()
  self._matrix = math.Matrix4()
  self._trackable = trackable
  self._frame = 0
  self._tf_frame = options.tf_frame or '0'
end

function Pose:update()
  if not self._trackable.pose_valid then return end

  self._frame = self._frame + 1
  if (self._frame % self._decimate) ~= 0 then return end

  self._trackable.pose:get_column(4, self._position)
  self._trackable.pose:to_quaternion(self._quaternion)
  self._topic:publish({
    header = {frame_id = self._tf_frame},
    pose = {position = self._position:to_dict3(),
            orientation = self._quaternion:to_dict()}
    })
end

function Pose:status()
  return self._trackable.device_class_name .. "|" .. self.name .. ":" .. self._tname
end

local ViveButtons = class("ViveButtons")
m.ViveButtons = ViveButtons

function ViveButtons:init(conn, trackable, options)
  local topic_name = string.format(options.topic, trackable.device_idx)
  print("Creating new ViveButtons publisher on " .. topic_name)
  self._tname = topic_name
  self._topic = conn:topic({
    topicName = topic_name,
    messageType = "sensor_msgs/Joy",
    queueSize = options.queue_size or 10
  })
  self._decimate = options.decimate or 9 -- default to 10fps publishing
  self._trackable = trackable
  self._frame = 0
end

function ViveButtons:update()
  local msg = {buttons = {0,0,0,0}, axes = {0.0, 0.0, 0.0}}
  local t = self._trackable
  msg.buttons[1] = t.buttons.ApplicationMenu or 0
  msg.buttons[2] = t.buttons.SteamVR_Trigger or 0
  msg.buttons[3] = t.buttons.SteamVR_Touchpad or 0
  msg.buttons[4] = t.buttons.Grip or 0
  msg.axes[1] = t.axes.trackpad1.x
  msg.axes[2] = t.axes.trackpad1.y
  msg.axes[3] = t.axes.trigger1.x
  self._topic:publish(msg)
end

ViveButtons.status = Pose.status

local Multi = class("Multi")
m.Multi = Multi

function Multi:init(conn, trackable, pubs)
  self._pubs = {}
  for _, opts in ipairs(pubs) do
    local p = opts.publisher(conn, trackable, opts)
    if p then table.insert(self._pubs, p) end
  end
end

function Multi:update()
  for _, pub in ipairs(self._pubs) do pub:update() end
end

function Multi:status()
  return self._trackable.device_class_name .. "|" .. self.name
end

local ROSConnection = class("ROSConnection")
m.ROSConnection = ROSConnection

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

-- A connection that logs to either a file or the regular log
local FileConnection = class("FileConnection")
m.FileConnection = FileConnection

local FileTopic = class("FileTopic")

function FileConnection:init(url)
  -- todo
  self.url = url or "LOG"
end

function FileConnection:is_connected()
  return true -- assume we can always open the file?
end

function FileConnection:status()
  return "File: " .. (self.url or "none")
end

function FileConnection:update()
  -- nothing to do
end

-- A plain websocket connection (for non-ros use cases)
local WSConnection = class("WSConnection")
m.WSConnection = WSConnection

-- 'forward' declaration
local WSTopic = class("WSTopic")

function WSConnection:init(url)
  local websocket = require("io/websocket.t")
  self.url = url
  self.socket = websocket.WebSocketConnection()
  if url then self.socket:connect(url) end
end

function WSConnection:is_connected()
  return self.socket and self.socket.open
end

function WSConnection:status()
  if not self.url then
    return "WS: No URL Specified"
  elseif self:is_connected() then
    return "WS: " .. self.url
  else
    return "WS: Disconnected"
  end
end

function WSConnection:topic(opts)
  return WSTopic(opts, self)
end

function WSConnection:update()
  if self.socket then self.socket:update() end
end

function WSTopic:init(options, conn)
  self.conn = conn
  self.opts = options
end

function WSTopic:publish(msg)
  local newmsg = {
    topic = self.opts.topicName,
    data = msg
  }
  self.conn.socket:sendJSON(newmsg)
end

return m
