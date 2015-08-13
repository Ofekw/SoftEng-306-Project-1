define mylaser ranger
(
  sensor(
  range [ 0.0 5.0 ] # range
  fov 180       # field of vision
  samples 180   # samples 
  )
  # generic model properties
  color "black"
  size [ 0.5 0.05 0.1 ]
  
)

define visitor position 
(
  color "gray50"
  drive "diff"		 	# Differential steering model.

  obstacle_return 1           	# Can hit things.
  laser_return 1                 # reflects laser beams
  ranger_return 1             	# reflects sonar beams
  blobfinder_return 1         	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"         	  

  size [1 1 3]
  #origin [-0.25 -0.25 0 0 ]

  mylaser(pose [ 0.000 0.000 -2.5 0.000 ])
  velocity_bounds [-1000 1000 -1000 1000 -1000 1000 -90 90 ]

  # left foot
  block
  (
	points 4
	point[0] [0 0.6]
	point[1] [0 1]
	point[2] [0.7 1]
	point[3] [0.7 0.6]
	z [0 0.05]
	color "brown"
  )

  # right foot
  block
  (
	points 4
	point[0] [0 0]
	point[1] [0 0.4]
	point[2] [0.7 0.4]
	point[3] [0.7 0]
	z [0 0.05]
	color "brown"
  )

  # left leg
  block
  (
	points 4
	point[0] [0 0.6]
	point[1] [0 1]
	point[2] [0.6 1]
	point[3] [0.6 0.6]
	z [0.05 0.4]
	color "blue"
  )

  # left leg
  block
  (
	points 4
	point[0] [0 0]
	point[1] [0 0.4]
	point[2] [0.6 0.4]
	point[3] [0.6 0]
	z [0.05 0.4]
	color "blue"
  )

  # torso
  block
  (
	points 4
	point[0] [0 0]
	point[1] [0 1]
	point[2] [0.6 1]
	point[3] [0.6 0]
	z [0.4 0.8]
	color "red"
  )

  # neck
  block
  (
	points 4
	point[0] [0 0.33]
	point[1] [0 0.66]
	point[2] [0.6 0.66]
	point[3] [0.6 0.33]
	z [0.8 0.85]
	color "pink"
  )

  # head
  block
  (
	points 4
	point[0] [0 0.23]
	point[1] [0 0.76]
	point[2] [0.6 0.76]
	point[3] [0.6 0.23]
	z [0.85 1]
	color "pink"
  )
)
