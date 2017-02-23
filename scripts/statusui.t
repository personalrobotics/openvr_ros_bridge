-- statusui.t
--
-- shows status

local nvg = require("graphics/nanovg.t")
local entity = require("ecs/entity.t")

local m = {}

local StatusComp = nvg.NanoVGDrawable:extend("StatusComp")
function StatusComp:init()
  self.mount_name = "status"
  self.lines = {"Line1", "Line2", "Line3", "etc."}
  self.fsize = 32
  self.linespacing = self.fsize * 1.25
  self.x0 = 50
  self.y0 = 100
end

function StatusComp:_create_colors(ctx)
  self.colors = {
    bg = ctx:RGBA(0, 0, 0, 128),
    fg = ctx:RGB(255,255,255)
  }
end

function StatusComp:nvg_draw(stage, ctx)
  local font = ctx:load_font("font/SourceCodePro-Regular.ttf", "sans")
  if not self.colors then self:_create_colors(ctx) end

  ctx:FontFaceId(font)
  ctx:FillColor(self.colors.fg)
  ctx:FontSize(self.fsize)
  for idx, line in ipairs(self.lines) do
    local y = self.y0 + (idx-1)*self.linespacing
    ctx:Text(self.x0, y, line, nil)
  end
end

function m.create_ui()
  return entity.Entity3d("ui_display", StatusComp())
end

return m
