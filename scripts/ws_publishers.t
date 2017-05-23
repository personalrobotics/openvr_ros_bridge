-- ws_publishers.t
--
-- websocket publishers

local m = {}
local class = require("class")
local publishers = require("publishers.t")
local ros_publishers = require("ros_publishers.t")

-- Reuse the ros publishers
m.Pose = ros_publishers.Pose
m.ViveButtons = ros_publishers.ViveButtons

-- A plain websocket connection (for non-ros use cases)
local WSConnection = class("WSConnection")
m.WSConnection = WSConnection
m.Connection = WSConnection -- alias this

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
    topic = self.opts.topic_name,
    data = msg
  }
  self.conn.socket:sendJSON(newmsg)
end

return m
