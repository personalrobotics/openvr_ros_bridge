-- file_publishers.t
--
-- dump pose to a file or the log

local m = {}
local publishers = require("publishers.t")

local Pose = publishers.Pose:extend("FilePose")
m.Pose = Pose

function Pose:init(conn, trackable, options)
  Pose.super.init(self, conn, trackable, options)

  self.divider = options.divider or ", "
  self.data_order = options.data_order or {"position", "quaternion"}
  for idx, v in ipairs(self.data_order) do
    self.data_order[idx] = "_" .. v
  end
  self.precision = options.precision or 4
  self.format_string = options.format or ("%." .. self.precision .. "f")

  self._dname = string.format(options.topic or "pose_%d", trackable.device_idx)
  self.topic = conn:topic(self._dname .. self.divider)
end

function Pose:format()
  local parts = {}
  for _, field_name in ipairs(self.data_order) do
    local field = self[field_name]
    local data = field:to_dict()
    for _, dataval in ipairs(data) do
      local s = string.format(self.format_string, dataval)
      table.insert(parts, s)
    end
  end
  return table.concat(parts, self.divider)
end

function Pose:publish()
  self.topic:publish(self:format())
end

-- A connection that logs to either a file or the regular log
local FileConnection = class("FileConnection")
m.FileConnection = FileConnection
m.Connection = FileConnection

local FileTopic = class("FileTopic")

function FileConnection:init(url)
  -- todo
  self.url = url or "LOG"
  if self.url and self.url ~= "LOG" then
    self.file, self.err = io.open(url, "w")
  end
end

function FileConnection:is_connected()
  return self.file or self.url == "LOG"
end

function FileConnection:status()
  return "File: " .. (self.url or "none")
end

function FileConnection:update()
  -- nothing to do
end

function FileConnection:topic(prefix)
  return FileTopic(self, prefix)
end

function FileConnection:write(data)
  if self.file then
    self.file:write(data .. "\n")
  elseif self.url == "LOG" then
    log.info(data)
  end
end

function FileTopic:init(parent, prefix)
  self.parent = parent
  self.prefix = prefix
end

function FileTopic:publish(msg)
  self.parent:write(self.prefix .. msg)
end

return m
