define laser ranger
(
  sensor(
    # laser-specific properties
    # factory settings for hokuyo LMS200	
    range [ 0.0  1.5 ]
    fov 360.0
    samples 360

  )
  # generic model properties
  color "blue"
  size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet	
)

define laser2 ranger
(
  sensor(
    # laser-specific properties
    # factory settings for hokuyo LMS200	
    range [ 0.0  5.0 ]
    fov 180.0
    samples 800
  )
  # generic model properties
  color "blue"
  size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet	
)

define laser3 ranger
(
  sensor(
    # laser-specific properties
    # factory settings for hokuyo LMS200	
    range [ 0.0  5.0 ]
    fov 180.0
    samples 800
  )
  # generic model properties
  color "blue"
  size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet	
)


define depth ranger
(
  sensor
  (
    range_max 6.5
    fov 58.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define sonars sensor
(
  # define the size of each transducer [xsize ysize zsize] in meters
  size [0.01 0.05 0.01 ] 
  # define the range bounds [min max]
  range [0 2.0]
  # define the angular field of view in degrees
  fov 15
  # define the number of samples spread over the fov
  samples 1
)

define sonar ranger
(
 # 8 transducers spread about the robot as follows
  #sonars( pose [ 0.075 0.130 0 30 ] )
  sonars( pose [ 0.0 0.0 0 0 ] )
  #sonars( pose [ 0.150 0.080 0 -30 ] )
 )
