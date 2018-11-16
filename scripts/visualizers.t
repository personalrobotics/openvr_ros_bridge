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
  root:add(line_entity)
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
