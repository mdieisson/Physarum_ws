# time to pause (in GUI mode) or quit (in headless mode (-g)) the simulation
quit_time 3600 # 1 hour of simulated time
paused 1


# configure the GUI window
window
(
  size [ 635.000 666.000 ] # in pixels
  scale 36.995   # pixels per meter
  center [ -0.0  -0.0 ]
  rotate [ 0  0 ]
  show_data 1              # show robot sensors 1=on 0=off
  show_footprints 0        # show robot shadow 1=on 0=off #rastro ou sombra do robô
)

define floorplan model
(
  # sombre, sensible, artistic
  
  color "gray30"
  size [100.0 100.0]
  resolution 0.16

  # most maps will need a bounding box
  boundary 0

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  ranger_return 0.0
  

  obstacle_return 1
)
