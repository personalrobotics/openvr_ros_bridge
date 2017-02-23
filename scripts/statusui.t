-- statusui.t
--
-- shows status

local nvg = require("graphics/nvg")
local entity = require("ecs/entity.t")

local m = {}

local StatusComp = nvg.NanoVGDrawable:extend("StatusComp")
function StatusComp:init()
  self.mount_name = "statusui"
end

function StatusComp:nvg_draw()
  -- bleh
end

function m.create_ui()
  local ret = nvg.NanoVGEntity("bridge_ui", function(comp, stage, ctx)
    -- bleh
  end)
end

return m
