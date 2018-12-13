-- bridge.t
--
-- openvr_ros_bridge main file

local VRTrackingApp = require("vr/vrtrackingapp.t").VRTrackingApp

local gfx = require("gfx")
local math = require("math")
local ecs = require("ecs")
local graphics = require("graphics")
local geometry = require("geometry")

local pbr = require("material/pbr.t")
local orbitcam = require("gui/orbitcam.t")
local grid = require("graphics/grid.t")

local openvr = require("vr/openvr.t")
local vrcomps = require("vr/components.t")

local ros = require("io/ros.t")
local statusui = require("statusui.t")

local conn = nil

function init()
  load_config()

  if config.Connection then
    conn = config.Connection(truss.args[2])
  end

  app = VRTrackingApp{
    title = "openvr_ros_bridge",
    width = config.width or 1280,
    height = config.height or 720,
    msaa = true,
    stats = config.stats ~= false,
    vsync = (config.rate or 60) <= 60,
    clear_color = 0x404080ff,
    auto_create_controllers = false
  }

  app.camera:add_component(orbitcam.OrbitControl({min_rad = 1, max_rad = 10}))

  create_scene(app.scene)
  openvr.on("trackable_connected", add_trackable)
  active_publishers = {}

  status = app.scene:create_child(statusui.StatusUI, "status")
end

local t0 = truss.tic()
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
  if config.rate and config.rate > 60 then
    local dt_ms = truss.toc(t0) * 1000.0
    local expected_dt_ms = 1000.0 / config.rate
    if dt_ms < expected_dt_ms then truss.sleep(expected_dt_ms - dt_ms) end
    t0 = truss.tic()
  end
end

function load_config()
  local fn = "config/" .. (truss.args[3] or "default.t")
  config = require(fn)
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
      trackable.publisher = pub
    else
      print("Trackable [" .. device_class .. "] did not produce a publisher.")
      print("Nothing will be published for this device.")
    end
  else
    print("Trackable [" .. device_class .. "] has no configured publisher.")
    print("Nothing will be published for this device.")
  end

  local display = cfg and cfg.display
  if display == true then
    local vis = require("visualizers.t")
    add_visualizers(trackable, {vis.BasicModel()})
  elseif display then
    add_visualizers(trackable, display)
  end
end

----------------------------------------------------------------------------
--- Graphics setup
----------------------------------------------------------------------------

-- create a big red ball so that there's something to see at least
function create_scene(root)
  local axis_geo = geometry.axis_widget_geo{scale = 0.5, length = 0.5}
  local axis_mat = pbr.FacetedPBRMaterial{
    diffuse = {0.2,0.03,0.01,1.0},
    tint = {0.001, 0.001, 0.001}, 
    roughness = 0.7
  }
  local thegrid = root:create_child(grid.Grid, "grid", { 
    spacing = 0.5, numlines = 8,
    color = {0.8, 0.8, 0.8}, thickness = 0.006
  })
  thegrid.quaternion:euler({x = -math.pi / 2.0, y = 0, z = 0}, 'ZYX')
  thegrid:update_matrix()
  root:create_child(graphics.Mesh, "mesh", axis_geo, axis_mat)
end

-- adds in visualizers
function add_visualizers(trackable, visualizers)
  local root = app.scene
  local trackable_entity = root:create_child(ecs.Entity3d, "vis")
  trackable_entity:add_component(vrcomps.TrackableComponent(trackable))
  
  for _, vis_constructor in ipairs(visualizers) do
    vis_constructor(root, trackable_entity, trackable)
  end
end
