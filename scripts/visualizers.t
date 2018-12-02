-- visualizers.t
--
-- visualization utilities

local class = require("class")
local math = require("math")
local graphics = require("graphics")
local geometry = require("geometry")
local ecs = require("ecs")

local m = {}

local function make_factory(constructor)
  return function(options)
    return function(root, entity, trackable)
      return constructor(root, entity, trackable, options)
    end
  end
end

local g_ui_ypos = 100

local UIComp = graphics.NanoVGComponent:extend("UIComp")
function UIComp:init(options)
  self.mount_name = "ui"
  self.fsize = options.font_size or 24
  self.linespacing = self.fsize * 1.1
  self.x0 = 50
  self.y0 = g_ui_ypos
  self.height = options.height or (self.linespacing * 1.5)
  self.prop = options.prop or "frame"
  self.trackable = options.trackable
  if options.format then
    if type(options.format) == 'string' then
      local fstring = options.format
      self.format = function(v)
        return string.format(fstring, v)
      end
    elseif type(options.format) == 'function' then
      self.format = options.format
    else
      truss.error("Format must be a format string or a function!")
    end
  end
  g_ui_ypos = g_ui_ypos + self.height
  UIComp.super.init(self)
end

function UIComp:nvg_setup(ctx)
  self.colors = {
    bg = ctx:RGBA(0, 0, 0, 128),
    fg = ctx:RGB(255,255,255)
  }
end

function UIComp:nvg_draw(ctx)
  local font = ctx:load_font("font/SourceCodePro-Regular.ttf", "sans")
  if not self.colors then self:nvg_setup(ctx) end

  local line = "?"
  if self.trackable and self.trackable.publisher then
    local p = self.trackable.publisher:get_prop(self.prop)
    local format = self.format or tostring
    if p then line = format(p) end
  end

  local hx = ctx.width / 2.0

  ctx:FontFace("sans")
  ctx:FillColor(self.colors.fg)
  ctx:FontSize(self.fsize)
  ctx:Text(hx + self.x0, self.y0, line, nil)
end

m.PropDisplay = make_factory(function(root, target_entity, trackable, options)
  options = options or {}
  options.trackable = trackable
  local display = root:create_child(ecs.Entity3d, "ui_display")
  display:add_component(UIComp(options))
end)

-- simply attach a mesh drawable directly to the entity
local axis_widget_geo = nil
m.BasicModel = make_factory(function(root, target_entity, trackable, options)
  options = options or {}
  if not axis_widget_geo then
    axis_widget_geo = geometry.axis_widget_geo{}
  end
  local pbr = require("material/pbr.t")
  local mat_options = options.material or options 
  local mat = pbr.FacetedPBRMaterial{
    diffuse = mat_options.diffuse   or {0.03,0.03,0.03,1.0}, 
    tint = mat_options.specular  or {0.001, 0.001, 0.001}, 
    roughness = mat_options.roughness or 0.7
  }

  target_entity:add_component(graphics.MeshComponent(axis_widget_geo, mat))
  target_entity.trackable:load_geo_to_component("mesh")
end)

-- foward declare this
local LineHistoryComponent = graphics.LineRenderComponent:extend("LineHistoryComponent")

-- the line history entity is attached to *root* rather than the trackable's
-- entity, because it makes more sense to keep history in the world space
m.LineHistory = make_factory(function(root, target_entity, trackable, options)
  local line_entity = root:create_child(ecs.Entity3d, "historyline")
  line_entity:add_component(LineHistoryComponent(options, target_entity))
end)

function LineHistoryComponent:init(options, target)
  options = options or {}
  local maxpoints = options.history_length or 900
  LineHistoryComponent.super.init(self, {maxpoints = maxpoints, dynamic = true})
  self.history_target = target
  self.target_pos = math.Vector()
  self.history_points = {}
  self.history_length = maxpoints
  self.decimate = options.decimate or 3
  self.frame_idx = 0
  self.history_filled = false
  self.mat.uniforms.u_baseColor:set(options.color or {0.8,0.3,0.3})
  self.mat.uniforms.u_thickness:set({options.thickness or 0.02})
end

function LineHistoryComponent:push_point(pt)
  for i = 1, self.history_length - 1 do
    self.history_points[i] = self.history_points[i+1] or pt
  end
  self.history_points[self.history_length] = pt
  self:set_points({self.history_points}) -- line expects a list of lists
end

function LineHistoryComponent:mount()
  LineHistoryComponent.super.mount(self)
  self:add_to_systems({"update"})
  self:wake()
end

function LineHistoryComponent:update()
  if self.frame_idx % self.decimate == 0 then
    self.history_target.matrix:get_column(4, self.target_pos)
    self:push_point(self.target_pos:to_array())
  end
  self.frame_idx = self.frame_idx + 1
end

return m
