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

function init()
  ROS = ros.Ros()
  local connected = ROS:connect(truss.args[3])
  if not connected then
    truss.quit()
    return
  end

  load_config()
  app = VRApp({title = "openvr_ros_bridge",
               mirror = "left"})
  create_scene(app.ECS.scene)
  openvr.on("trackable_connected", add_trackable)
  active_publishers = {}
end

function update()
  app:update()
  update_publishers()
  ROS:update()
end

function load_config()
  config = require(truss.args[4] or "config.t")
end

function update_publishers()
  for _, pub in pairs(active_publishers) do pub:update() end
end

function add_trackable(trackable)
  local device_class = trackable.device_class_name
  local cfg = config[device_class]
  if cfg and cfg.publisher then
    local pub = cfg.publisher(ROS, trackable, cfg)
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

  if device_class == "Controller" then
    add_controller_model(trackable)
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

  local thingy = entity.Entity3d()
  thingy.position:set(0.0, 0.5, 0.0)
  thingy:update_matrix()
  thingy:add_component(pipeline.MeshShaderComponent(geo, mat))
  root:add(thingy)
end

-- adds the controller model in
function add_controller_model(trackable)
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
  controller:add_component(vrcomps.VRControllerComponent(trackable))
  controller.vr_controller:load_geo_to_component("mesh_shader")

  app.ECS.scene:add(controller)
end
