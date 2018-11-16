-- statusui.t
--
-- shows status

local graphics = require("graphics")
local ecs = require("ecs")

local m = {}

local StatusComp = graphics.NanoVGComponent:extend("StatusComp")
function StatusComp:init()
  self.mount_name = "status"
  self.lines = {"Line1", "Line2", "Line3", "etc."}
  self.fsize = 32
  self.linespacing = self.fsize * 1.25
  self.x0 = 50
  self.y0 = 100
  StatusComp.super.init(self)
end

function StatusComp:nvg_setup(ctx)
  self.colors = {
    bg = ctx:RGBA(0, 0, 0, 128),
    fg = ctx:RGB(255,255,255)
  }
end

function StatusComp:nvg_draw(ctx)
  local font = ctx:load_font("font/SourceCodePro-Regular.ttf", "sans")
  if not self.colors then self:_create_colors(ctx) end

  ctx:FontFace("sans")
  ctx:FillColor(self.colors.fg)
  ctx:FontSize(self.fsize)
  for idx, line in ipairs(self.lines) do
    local y = self.y0 + (idx-1)*self.linespacing
    ctx:Text(self.x0, y, line, nil)
  end
end

function m.StatusUI(_ecs, name)
  return ecs.Entity3d(_ecs, name, StatusComp())
end

return m
