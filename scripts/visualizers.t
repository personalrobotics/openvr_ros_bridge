-- visualizers.t
--
-- visualization utilities

local class = require("class")
local math = require("math")
local pipeline = require("graphics/pipeline.t")
local line = require("graphics/line.t")
local entity = require("ecs/entity.t")

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
    axis_widget_geo = require("geometry/widgets.t").axis_widget_geo("axis", 0.4, 0.2, 6)
  end
  local pbr = require("shaders/pbr.t")
  local mat_options = options.material or {}
  local diffuse   = mat_options.diffuse   or {0.03,0.03,0.03,1.0}
  local tint      = mat_options.specular  or {0.001, 0.001, 0.001}
  local roughness = mat_options.roughness or 0.7
  local mat = pbr.FacetedPBRMaterial(diffuse, tint, roughness)

  target_entity:add_component(pipeline.MeshShaderComponent(axis_widget_geo, mat))
  target_entity.vr_trackable:load_geo_to_component("mesh_shader")
end)

-- foward declare this
local LineHistoryComponent = line.LineShaderComponent:extend("LineHistoryComponent")

-- the line history entity is attached to *root* rather than the trackable's
-- entity, because it makes more sense to keep history in the world space
m.LineHistory = make_factory(function(root, target_entity, trackable, options)
  local line_entity = entity.Entity3d()
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
  self.mat.uniforms.u_color:set(options.color or {0.8,0.3,0.3})
  self.mat.uniforms.u_thickness:set({options.thickness or 0.005})
end

function LineHistoryComponent:push_point(pt)
  for i = 1, self.history_length - 1 do
    self.history_points[i] = self.history_points[i+1] or pt
  end
  self.history_points[self.history_length] = pt
  self:set_points({self.history_points}) -- line expects a list of lists
end

function LineHistoryComponent:on_update()
  if self.frame_idx % self.decimate == 0 then
    self.history_target.matrix:get_column(4, self.target_pos)
    self:push_point(self.target_pos:to_array())
  end
  self.frame_idx = self.frame_idx + 1
  LineHistoryComponent.super.on_update(self)
end

return m
