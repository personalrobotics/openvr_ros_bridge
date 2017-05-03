-- bridge.t
--
-- openvr_ros_bridge main file

local VRApp = require("vr/vrapp.t").VRApp
local icosphere = require("geometry/icosphere.t")
local gfx = require("gfx")
local math = require("math")
local entity = require("ecs/entity.t")
local pipeline = require("graphics/pipeline.t")
local openvr = require("vr/openvr.t")
local vrcomps = require("vr/components.t")
local ros = require("io/ros.t")
local grid = require("graphics/grid.t")
local statusui = require("statusui.t")

local conn = nil

function init()
  load_config()

  if config.Connection then
    conn = config.Connection(truss.args[3])
  end

  app = VRApp({title = "openvr_ros_bridge", nvg = true,
               mirror = "left", debugtext = true})
  create_scene(app.ECS.scene)
  openvr.on("trackable_connected", add_trackable)
  active_publishers = {}

  status = statusui.create_ui()
  app.ECS.scene:add(status)
end

function update()
  app:update()
  status.status.lines = {}
  if conn then
    status.status.lines[1] = conn:status()
  else
    status.status.lines[1] = "[no connection]"
  end
  update_publishers(status.status.lines)
  if conn then conn:update() end
end

function load_config()
  config = require(truss.args[4] or "config.t")
end

function update_publishers(statuslines)
  local templines = {}
  for idx, pub in pairs(active_publishers) do
    pub:update()
    templines[idx] = string.format("%2d|%s", idx, pub:status())
  end
  for i = 0,openvr.MAX_TRACKABLES do
    local line = templines[i]
    if not line then
      if openvr.trackables[i+1] then
        line = string.format("%2d|%s|[unpublished]",
                             i, openvr.trackables[i+1].device_class_name)
      else
        line = string.format("%2d|[none]", i)
      end
    end
    statuslines[i+2] = line
  end
end

function add_trackable(trackable)
  local device_class = trackable.device_class_name
  local cfg = config[device_class]
  if conn and conn:is_connected() and cfg and cfg.publisher then
    local pub = cfg.publisher(conn, trackable, cfg)
    if pub then
      active_publishers[trackable.device_idx] = pub
    else
      print("Trackable [" .. device_class .. "] did not produce a publisher.")
      print("Nothing will be published for this device.")
    end
  else
    print("Trackable [" .. device_class .. "] has no configured publisher.")
    print("Nothing will be published for this device.")
  end

  if cfg and cfg.display then
    print("Adding model?")
    add_trackable_model(trackable)
  end
end

----------------------------------------------------------------------------
--- Graphics setup
----------------------------------------------------------------------------

function create_uniforms()
  local uniforms = gfx.UniformSet()
  uniforms:add(gfx.VecUniform("u_baseColor"))
  uniforms:add(gfx.VecUniform("u_pbrParams"))
  uniforms:add(gfx.VecUniform("u_lightDir", 4))
  uniforms:add(gfx.VecUniform("u_lightRgb", 4))

  uniforms.u_lightDir:set_multiple({
          math.Vector( 1.0,  1.0,  0.0),
          math.Vector(-1.0,  1.0,  0.0),
          math.Vector( 0.0, -1.0,  1.0),
          math.Vector( 0.0, -1.0, -1.0)})

  uniforms.u_lightRgb:set_multiple({
          math.Vector(0.8, 0.8, 0.8),
          math.Vector(1.0, 1.0, 1.0),
          math.Vector(0.1, 0.1, 0.1),
          math.Vector(0.1, 0.1, 0.1)})

  uniforms.u_baseColor:set(math.Vector(0.2,0.03,0.01,1.0))
  uniforms.u_pbrParams:set(math.Vector(0.001, 0.001, 0.001, 0.7))
  return uniforms
end

-- create a big red ball so that there's something to see at least
function create_scene(root)
  local geo = icosphere.icosphere_geo(1.0, 3, "ico")
  local mat = {
    state = gfx.create_state(),
    uniforms = create_uniforms(),
    program = gfx.load_program("vs_basicpbr", "fs_basicpbr_faceted_x4")
  }
  sphere_geo = geo
  sphere_mat = mat

  local thegrid = grid.Grid({ spacing = 0.5, numlines = 8,
                              color = {0.8, 0.8, 0.8}, thickness = 0.003})
  thegrid.quaternion:euler({x= -math.pi / 2.0, y=0, z=0}, 'ZYX')
  thegrid:update_matrix()
  root:add(thegrid)

  local axis_geo = require("geometry/widgets.t").axis_widget_geo("axis", 0.4, 0.2, 6)
  root:add(pipeline.Mesh("axis0", axis_geo, mat))
end

-- adds the controller model in
function add_trackable_model(trackable)
  local geo = icosphere.icosphere_geo(0.1, 3, "cico")
  local m2 = {
    state = sphere_mat.state,
    program = sphere_mat.program,
    uniforms = sphere_mat.uniforms:clone()
  }
  m2.uniforms.u_baseColor:set(math.Vector(0.03,0.03,0.03,1.0))
  m2.uniforms.u_pbrParams:set(math.Vector(0.001, 0.001, 0.001, 0.7))

  local controller = entity.Entity3d()
  controller:add_component(pipeline.MeshShaderComponent(geo, m2))
  controller:add_component(vrcomps.VRTrackableComponent(trackable))
  controller.vr_trackable:load_geo_to_component("mesh_shader")

  app.ECS.scene:add(controller)
end
