define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  #size [ 0.2552 0.2552 0.40 ]
  size [1.5 0.4 0.3]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
)

define roomba position
(
  size [0.33 0.33 0.1]

  # this block approximates the circular shape of a Roomba
  block( 
    points 16
    point[0] [ 0.225 0.000 ]
    point[1] [ 0.208 0.086 ]
    point[2] [ 0.159 0.159 ]
    point[3] [ 0.086 0.208 ]
    point[4] [ 0.000 0.225 ]
    point[5] [ -0.086 0.208 ]
    point[6] [ -0.159 0.159 ]
    point[7] [ -0.208 0.086 ]
    point[8] [ -0.225 0.000 ]
    point[9] [ -0.208 -0.086 ]
    point[10] [ -0.159 -0.159 ]
    point[11] [ -0.086 -0.208 ]
    point[12] [ -0.000 -0.225 ]
    point[13] [ 0.086 -0.208 ]
    point[14] [ 0.159 -0.159 ]
    point[15] [ 0.208 -0.086 ]
    z [0 0.1]
  )

  localization "gps"
      localization_origin [0 0 0 0]

  color "gray50"
)


define roomba2 position
( 
  size [1.5 0.4 0.3]

  # this block approximates the circular shape of a Roomba
  block( 
    points 16
    point[0] [ 0.225 0.000 ]
    point[1] [ 0.208 0.086 ]
    point[2] [ 0.159 0.159 ]
    point[3] [ 0.086 0.208 ]
    point[4] [ 0.000 0.225 ]
    point[5] [ -0.086 0.208 ]
    point[6] [ -0.159 0.159 ]
    point[7] [ -0.208 0.086 ]
    point[8] [ -0.225 0.000 ]
    point[9] [ -0.208 -0.086 ]
    point[10] [ -0.159 -0.159 ]
    point[11] [ -0.086 -0.208 ]
    point[12] [ -0.000 -0.225 ]
    point[13] [ 0.086 -0.208 ]
    point[14] [ 0.159 -0.159 ]
    point[15] [ 0.208 -0.086 ]
    z [0 0.1]
  )

  localization "gps"
      localization_origin [0 0 0 0]

  color "gray50"
)

define caminhao position
(
  pose [0.0 0.0 0.0 0.0]


  size [0.75 0.3 0.3]
  origin [0.0 0.0 0.0 0.0]
  gui_nose 1
  drive "diff"
  color "gray"
  obstacle_return 1
  fiducial_return 1
)


define caixa model
(
  pose [0 0 0.4 0] 
  size [0.75 0.3 0.4]
  color "brown"
  gui_nose 0
  obstacle_return 0
  fiducial_return 0
  name "caixa_0"  # nome único para cada uma

)

define containertotal model
(
  pose [-3.7 1 0.4 0] 
  size [1.0 4.8 0.4]
  color "brown"
  gui_nose 0
  obstacle_return 0
  fiducial_return 0
  name "container_0"  # nome único para cada uma

)