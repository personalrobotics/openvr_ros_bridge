-- publishers.t
--
-- base publishers

local m = {}
local class = require("class")
local math = require("math")

local Pose = class("Pose")
m.Pose = Pose

function Pose:init(conn, trackable, options)
  self._decimate = options.decimate or 9 -- default to 10fps publishing
  self._position = math.Vector()
  self._velocity = math.Vector()
  self._angular_velocity = math.Vector()
  self._quaternion = math.Quaternion()
  self._matrix = math.Matrix4()
  self._trackable = trackable
  self._frame = 0
  self._time = 0.0
  self._t0 = truss.tic()
end

function Pose:update()
  if not self._trackable.pose_valid then return end

  self._frame = self._frame + 1
  if (self._frame % self._decimate) ~= 0 then return end

  self._time = truss.toc(self._t0)
  self._trackable.pose:get_column(4, self._position)
  self._trackable.pose:to_quaternion(self._quaternion)
  self._velocity = self._trackable.velocity
  self._angular_velocity = self._trackable.angular_velocity
  self:publish()
end

function Pose:status()
  return self._trackable.device_class_name .. " | " .. (self._dname or self.name)
end

function Pose:get_prop(propname)
  return self["_" .. propname]
end

local ViveButtons = class("ViveButtons")
m.ViveButtons = ViveButtons

function ViveButtons:init(conn, trackable, options)
  self._decimate = options.decimate or 9 -- default to 10fps publishing
  self._trackable = trackable
  self._frame = 0
  self._msg = {buttons = {0,0,0,0}, axes = {0.0, 0.0, 0.0}}
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
  self:publish(msg)
  self._msg = msg
end

function ViveButtons:get_prop(propname)
  return self._msg[propname]
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

function Multi:get_prop(propname)
  for _, pub in ipairs(self._pubs) do
    local p = pub:get_prop(propname)
    if p then return p end
  end
end

return m
